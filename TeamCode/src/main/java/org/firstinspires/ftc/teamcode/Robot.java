package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.PoseFusion;

import java.util.List;

/**
 * Top-level composition: owns every subsystem and fans out one update per loop.
 *
 * <p><b>Loop ordering matters.</b> {@link #readSensors()} and {@link #writeActuators()} are separate
 * so an OpMode can do read -> decide -> act in that order. If actuators were written before the
 * scheduler ran, every command would be deciding on last loop's sensor data.
 */
@Configurable
public class Robot {
    /** Hue of a pollen game piece, in degrees. Yellow sits near 55. Tune on the real field. */
    public static float POLLEN_HUE_DEGREES = 55f;
    public static float POLLEN_HUE_TOLERANCE = 25f;
    public static float POLLEN_MIN_SATURATION = 0.45f;
    public static float POLLEN_MIN_VALUE = 0.20f;

    public final Drivetrain drivetrain;
    public final Limelight limelight;
    public final ColorSensor colorSensor;
    public final Intake intake;

    /** Multi-subsystem one-button actions. Owned here so every OpMode gets the same set. */
    public final Macros macros;

    /** Blends odometry with AprilTag fixes. See {@link #updateLocalization()}. */
    public final PoseFusion poseFusion = new PoseFusion();

    private final List<LynxModule> hubs;

    public Robot(HardwareMap hardwareMap) {
        Hardware.reset();

        // MANUAL bulk caching batches every encoder/current read into one bus transaction per loop.
        // Without it each getCurrent()/getVelocity() is its own USB round-trip, and the intake alone
        // is read three times per loop (anti-jam, telemetry, logger).
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        drivetrain = new Drivetrain(hardwareMap);
        limelight = new Limelight(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.setCapturedSupplier(this::pollenAtColorSensor);

        macros = new Macros(this);
    }

    /** Names of hardware devices missing from the robot configuration. Empty means all present. */
    public List<String> getMissingHardware() {
        return Hardware.getMissing();
    }

    /**
     * Refreshes cached sensor data. Call at the TOP of the loop, before commands run.
     *
     * <p>Clearing the bulk cache here is what makes the whole loop see one consistent snapshot.
     */
    public void readSensors() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        limelight.update();
        colorSensor.update();
    }

    /** Pushes queued outputs to hardware. Call at the BOTTOM of the loop, after commands run. */
    public void writeActuators() {
        intake.update();
        drivetrain.update();
    }

    /** Convenience for callers that do not need the read/write split (e.g. diagnostics). */
    public void update() {
        readSensors();
        writeActuators();
    }

    /**
     * Releases non-actuator hardware at the end of an OpMode. Safe to call more than once.
     *
     * <p>Deliberately does not write to motors. Iterative OpModes are forbidden from driving
     * actuators in {@code stop()} — the SDK rejects it as CANCELLED_FOR_SAFETY — and it zeroes them
     * for us regardless. Callers that need motors stopped mid-OpMode should call
     * {@link #writeActuators()} after {@code intake.stop()} themselves.
     */
    public void stop() {
        intake.stop();
        colorSensor.setLight(false);
        limelight.stop();
    }

    /** Cleanup shared by the natural end of a vision macro and by an operator abort. */
    public void abortMacro() {
        drivetrain.cancelPath();
        limelight.activateAprilTagPipeline();
        macros.markCancelled();
    }

    /**
     * True when the colour sensor is looking at something pollen-coloured.
     *
     * <p>Thresholds hue first, gated by saturation and brightness. A brightness-only test would
     * fire on any bright object — a white field wall reads as "captured".
     */
    private boolean pollenAtColorSensor() {
        return colorSensor.matchesHue(
                POLLEN_HUE_DEGREES, POLLEN_HUE_TOLERANCE, POLLEN_MIN_SATURATION, POLLEN_MIN_VALUE);
    }

    /**
     * Runs one step of odometry/vision fusion and writes the result back to the follower.
     *
     * <p>Call once per loop <em>during a match</em>. Unlike {@link #tryLocalizeFromAprilTag()}, which
     * hard-sets the pose and is only appropriate at init when there is no prior estimate, this
     * blends: a fix is gated on tag count, field bounds, and a maximum jump, then latency-compensated
     * and filtered. One bad frame nudges the estimate instead of teleporting the robot.
     *
     * <p>Does nothing while a path is running if the camera is on the pollen pipeline, since the
     * botpose is meaningless there.
     */
    public void updateLocalization() {
        Pose odometry = drivetrain.getPose();
        if (odometry == null) return;

        Pose vision = limelight.getBotposeAsPedroPose();   // already null unless it is trustworthy
        Pose corrected = poseFusion.update(
                System.currentTimeMillis(), odometry, vision, limelight.getVisionLatencyMs());

        // The fusion contract requires writing the result back - its delta bookkeeping assumes the
        // returned pose became the follower's new baseline.
        if (corrected != null) drivetrain.setPose(corrected);
    }

    /**
     * Attempts a one-shot AprilTag relocalisation, hard-setting the pose. Returns whether it worked.
     *
     * <p>For init only, when there is no prior estimate worth preserving. During a match use
     * {@link #updateLocalization()} instead, which blends rather than teleports.
     *
     * <p>Only succeeds on the AprilTag pipeline with at least one tag in view — {@link Limelight}
     * enforces both, because a default all-zeros botpose would otherwise teleport us to field centre.
     */
    public boolean tryLocalizeFromAprilTag() {
        if (limelight.getPipelineIndex() != Limelight.APRILTAG_PIPELINE_INDEX) {
            // The switch takes several frames to take effect; this call returns false until then
            // and the next loop iteration retries.
            limelight.activateAprilTagPipeline();
            return false;
        }
        Pose botpose = limelight.getBotposeAsPedroPose();
        if (botpose == null) return false;
        drivetrain.setPose(botpose);
        poseFusion.seed(botpose);
        return true;
    }

}
