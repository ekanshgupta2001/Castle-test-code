package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.subsystems.templates.ExampleLift;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.firstinspires.ftc.teamcode.util.diagnostics.Drawing;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;

/**
 * Main driver-controlled OpMode.
 *
 * <p>Driving is itself an Ivy command ({@code drivetrain.driverControlCommand}) rather than code in
 * the loop. That is what makes macros safe: scheduling one suspends driver control through the
 * scheduler, and ending or cancelling it restores driver control automatically. There is no
 * "am I in a macro?" flag for the loop to get wrong.
 *
 * <p>The lifecycle — robot construction, logging, loop timing, draw rate-limiting, the read/decide/
 * act ordering — lives in {@link MatchOpMode}. What remains here is only what makes this OpMode
 * teleop: the bindings, the haptics, and the two telemetry modes.
 *
 * <p>Every binding comes from {@link Controls}, which also generates the init-phase help card, so
 * the card cannot describe a button this class does not actually read.
 */
@Configurable
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends MatchOpMode {
    /** Stick deflection that counts as "the driver wants control back" and aborts a macro. */
    private static final double MACRO_ABORT_STICK = 0.25;

    /**
     * Whether to show the full engineering readout.
     *
     * <p>Off during a match. Nobody reads twenty lines of subsystem state while driving, and the
     * four things that matter — time, possession, what the last macro did, and whether anything is
     * broken — get lost among them.
     */
    public static boolean DEBUG_TELEMETRY = false;

    /** Below this, the pack is sagging enough to change how the robot drives. Warn the drivers. */
    public static double LOW_BATTERY_VOLTS = 11.5;

    /** Blips on a macro that succeeded. */
    private static final int RUMBLE_SUCCESS_BLIPS = 1;
    /** Blips on a macro that timed out or never saw a target — distinguishable without looking. */
    private static final int RUMBLE_FAILURE_BLIPS = 3;
    /** Blips when endgame begins. */
    private static final int RUMBLE_ENDGAME_BLIPS = 2;

    private boolean localized = false;
    private boolean inheritedPose = false;
    private Command activeMacro = null;

    // Previous values, for firing haptics on the transition rather than continuously.
    private Macros.Outcome lastOutcome = Macros.Outcome.IDLE;
    private boolean lastHadPiece = false;
    private boolean endgameAnnounced = false;

    @Override
    protected String logTag() {
        return "teleop";
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    @Override
    protected void onInit() {
        // Default commands. Both sit at priority -1 with SUSPEND, so any macro preempts them and
        // they resume by themselves when it finishes.
        robot.intake.defaultIdleCommand().schedule();
        robot.drivetrain.driverControlCommand(
                () -> DriveScaling.shape(-gamepad1.left_stick_y) * slowScale(),
                () -> DriveScaling.shape(-gamepad1.left_stick_x) * slowScale(),
                () -> DriveScaling.shape(-gamepad1.right_stick_x) * slowScale()
        ).schedule();

        // Inherit where autonomous left off. Without this, teleop starts with an unknown heading
        // while defaulting to field-centric drive - the mode that depends on heading most - so the
        // driver's first stick input sends the robot in an arbitrary direction.
        if (PoseStorage.hasPose()) {
            robot.drivetrain.setStartingPose(PoseStorage.getPose());
            robot.poseFusion.seed(PoseStorage.getPose());
            inheritedPose = true;
        }
    }

    @Override
    protected void onInitLoop() {
        // Re-evaluated every loop rather than latched: a reading going stale or a tag leaving view
        // should be visible to the drivers, not hidden behind a sticky "yes".
        localized = robot.tryLocalizeFromAprilTag();

        for (String m : robot.getMissingHardware()) telemetry.addLine("MISSING: " + m);
        telemetry.addData("Alliance", PoseStorage.hasAlliance() ? PoseStorage.getAlliance() : "unknown");
        telemetry.addData("Pose from auto?", inheritedPose ? "yes" : "no - drive is unreferenced");
        telemetry.addData("Localized?", localized ? "yes" : "looking for AprilTag...");
        telemetry.addData("Tags in view", robot.limelight.getBotposeTagCount());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addLine();
        for (String line : Controls.helpLines()) telemetry.addLine(line);
    }

    @Override
    protected void onStart() {
        robot.drivetrain.startTeleop();
    }

    @Override
    protected void onDecide() {
        handleDriverInput();
        handleOperatorInput();
    }

    @Override
    protected void onAfterAct() {
        updateHaptics();
    }

    private double slowScale() {
        return DriveScaling.slowScale(gamepad1.left_trigger);
    }

    // ---- Input ----

    private void handleDriverInput() {
        if (Controls.TOGGLE_DRIVE_FRAME.wasPressed(gamepad1, gamepad2)) {
            robot.drivetrain.toggleFieldCentric();
        }
        if (Controls.RESET_HEADING.wasPressed(gamepad1, gamepad2)) {
            // Escape hatch when field-centric drive has drifted: treat the current facing as
            // heading zero. Without this a bad localisation makes the robot undrivable.
            robot.drivetrain.resetHeading();
        }

        // Two ways out of a macro: an explicit abort button, or simply grabbing the sticks.
        if (macroRunning()
                && (Controls.ABORT.wasPressed(gamepad1, gamepad2) || driverWantsControl())) {
            abortMacro();
        }
        if (macroRunning()) return;

        if (Controls.COLLECT.wasPressed(gamepad1, gamepad2)) {
            startMacro(robot.macros.collectPiece());
        } else if (Controls.ALIGN_SERVO.wasPressed(gamepad1, gamepad2)) {
            startMacro(robot.macros.servoAlignToPiece());
        } else if (Controls.ALIGN_PATH.wasPressed(gamepad1, gamepad2)) {
            startMacro(robot.macros.alignToPiece());
        } else if (Controls.RELOCALIZE.wasPressed(gamepad1, gamepad2)) {
            startMacro(robot.macros.relocalize());
        } else if (Controls.SNAP_90.wasPressed(gamepad1, gamepad2)) {
            snapTo(90);
        } else if (Controls.SNAP_0.wasPressed(gamepad1, gamepad2)) {
            snapTo(0);
        } else if (Controls.SNAP_270.wasPressed(gamepad1, gamepad2)) {
            snapTo(270);
        } else if (Controls.SNAP_180.wasPressed(gamepad1, gamepad2)) {
            snapTo(180);
        }
    }

    private void handleOperatorInput() {
        if (Controls.INTAKE.wasPressed(gamepad1, gamepad2)) {
            robot.intake.intakeCommand().schedule();
        }
        if (Controls.OUTTAKE.wasPressed(gamepad1, gamepad2)) {
            robot.intake.outtakeCommand().schedule();
        }
        if (Controls.EJECT.wasPressed(gamepad1, gamepad2)) {
            robot.intake.ejectCommand().schedule();
        }
        if (Controls.STOP_INTAKE.wasPressed(gamepad1, gamepad2)) {
            robot.intake.stopCommand().schedule();
        }
        if (Controls.CAPTURE_AND_HOLD.wasPressed(gamepad1, gamepad2)) {
            robot.intake.captureAndHoldCommand().schedule();
        }

        // Inert until a lift exists in the configuration - isAvailable() gates every call.
        if (robot.lift.isAvailable()) {
            if (Controls.LIFT_HIGH.wasPressed(gamepad1, gamepad2)) {
                robot.lift.goToLevel(ExampleLift.Level.HIGH).schedule();
            }
            if (Controls.LIFT_LOW.wasPressed(gamepad1, gamepad2)) {
                robot.lift.goToLevel(ExampleLift.Level.LOW).schedule();
            }
            if (Controls.LIFT_DOWN.wasPressed(gamepad1, gamepad2)) {
                robot.lift.goToLevel(ExampleLift.Level.DOWN).schedule();
            }
            if (Controls.TOGGLE_GRIP.wasPressed(gamepad1, gamepad2)) {
                robot.lift.toggleGrip().schedule();
            }
        }

        if (Controls.TOGGLE_DEBUG.wasPressed(gamepad1, gamepad2)) {
            DEBUG_TELEMETRY = !DEBUG_TELEMETRY;
        }
    }

    private void snapTo(double degrees) {
        startMacro(robot.macros.snapToHeading(Math.toRadians(degrees)));
    }

    private void startMacro(Command macro) {
        activeMacro = macro;
        macro.schedule();
    }

    private boolean macroRunning() {
        return activeMacro != null && Scheduler.isScheduled(activeMacro);
    }

    private boolean driverWantsControl() {
        return Math.abs(gamepad1.left_stick_y) > MACRO_ABORT_STICK
                || Math.abs(gamepad1.left_stick_x) > MACRO_ABORT_STICK
                || Math.abs(gamepad1.right_stick_x) > MACRO_ABORT_STICK;
    }

    private void abortMacro() {
        Scheduler.cancel(activeMacro);
        // Cancelling the command releases the drivetrain resource, but the follower drives itself
        // once handed a path - this is the call that actually stops the robot.
        robot.abortMacro();
        activeMacro = null;
    }

    // ---- Feedback ----

    /**
     * Haptic feedback for things a driver cannot see.
     *
     * <p>Telemetry reports macro outcomes accurately and no driver reads it mid-match — which made
     * that reporting effectively write-only. Each of these fires on a <em>transition</em>, so a
     * held state never buzzes continuously.
     */
    private void updateHaptics() {
        Macros.Outcome outcome = robot.macros.getOutcome();
        if (outcome != lastOutcome) {
            if (outcome == Macros.Outcome.SUCCESS) {
                gamepad1.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
            } else if (outcome == Macros.Outcome.TIMED_OUT || outcome == Macros.Outcome.NO_TARGET) {
                gamepad1.rumbleBlips(RUMBLE_FAILURE_BLIPS);
            }
            // CANCELLED is deliberately silent: the driver just cancelled it and already knows.
            lastOutcome = outcome;
        }

        // Possession is the one piece of state both drivers act on, so both get told.
        boolean hasPiece = robot.intake.hasPiece();
        if (hasPiece && !lastHadPiece) {
            gamepad1.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
            gamepad2.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
        }
        lastHadPiece = hasPiece;

        MatchClock clock = robot.getMatchClock();
        if (!endgameAnnounced && clock != null && clock.isEndgame()) {
            endgameAnnounced = true;
            gamepad1.rumbleBlips(RUMBLE_ENDGAME_BLIPS);
            gamepad2.rumbleBlips(RUMBLE_ENDGAME_BLIPS);
        }
    }

    @Override
    protected void onDraw() {
        Drawing.drawRobot(robot.drivetrain.getPose());
        if (robot.limelight.hasStableBlob()) {
            Drawing.drawTarget(robot.limelight.estimateBlobApproachPose(robot.drivetrain.getPose()));
        }
        Drawing.sendPacket();
    }

    // ---- Telemetry ----

    @Override
    protected void onTelemetry() {
        matchTelemetry();
        if (DEBUG_TELEMETRY) debugTelemetry();
    }

    /**
     * What a driver can actually use mid-match: time, possession, what the last macro did, and
     * anything broken. Faults render only when present, so their presence is itself the signal.
     */
    private void matchTelemetry() {
        MatchClock clock = robot.getMatchClock();
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Carrying", robot.intake.hasPiece() ? "YES" : "no");
        telemetry.addData("Macro", robot.macros.getStatus());
        telemetry.addData("Drive", robot.drivetrain.isFieldCentric() ? "Field" : "Robot");

        for (String missing : robot.getMissingHardware()) {
            telemetry.addData("!! MISSING", missing);
        }
        if (loggerError != null) telemetry.addData("!! Logger FAILED", loggerError);
        if (robot.intake.hasGivenUpUnjamming()) {
            telemetry.addLine("!! INTAKE JAMMED - anti-jam gave up. Use "
                    + Controls.OUTTAKE.button() + " on gamepad 2 to outtake.");
        }
        double volts = robot.getBatteryVolts();
        if (volts > 0 && volts < LOW_BATTERY_VOLTS) {
            telemetry.addData("!! BATTERY LOW", "%.2f V", volts);
        }

        if (!DEBUG_TELEMETRY) {
            telemetry.addLine("\n(" + Controls.TOGGLE_DEBUG.button() + " on gamepad 2 for debug)");
        }
    }

    /** Everything else. Useful in the pit and at practice; noise during a match. */
    private void debugTelemetry() {
        telemetry.addLine();
        // Not just the instantaneous value: a spike lasts one cycle and is gone before anyone can
        // read it, so p95, max and a spike count are what actually diagnose a stuttering loop.
        telemetry.addData("Loop", loopStats.getStatus());
        telemetry.addData("Battery V", "%.2f", robot.getBatteryVolts());
        telemetry.addData("Slow scale", "%.2f", slowScale());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Heading hold", robot.drivetrain.isHeadingHoldActive()
                ? String.format("holding %.0f deg", Math.toDegrees(robot.drivetrain.getHeldHeading()))
                : "driver steering");
        telemetry.addData("Following path?", robot.drivetrain.isFollowingPath());
        telemetry.addLine();
        telemetry.addData("Intake mode", robot.intake.getMode());
        telemetry.addData("Intake target v", "%.0f", robot.intake.getTargetVelocity());
        telemetry.addData("Intake actual v", "%.0f", robot.intake.getVelocityTicksPerSec());
        telemetry.addData("Intake amps", "%.2f", robot.intake.getCurrentAmps());
        telemetry.addData("Unjamming?", robot.intake.isUnjamming());
        telemetry.addData("Unjam attempts", robot.intake.getUnjamAttempts());
        if (robot.lift.isAvailable()) {
            telemetry.addData("Lift", robot.lift.getLevel() + (robot.lift.isAtLevel() ? " (there)" : " ..."));
            telemetry.addData("Claw", robot.lift.getGrip());
        }
        telemetry.addLine();
        telemetry.addData("Localization", robot.poseFusion.getStatus());
        telemetry.addData("Pipeline", robot.limelight.getPipelineName());
        telemetry.addData("LL tags", robot.limelight.getBotposeTagCount());
        telemetry.addData("Blob lock?", robot.limelight.hasStableBlob());
        telemetry.addData("Blob spread", "%.1f deg", robot.limelight.getBlobSpreadDegrees());
        telemetry.addData("Blob range", "%.1f in", robot.limelight.estimateBlobDistanceInches());
        telemetry.addData("Color hue", "%.0f", robot.colorSensor.getHue());
        telemetry.addData("Color sat/val", "%.2f / %.2f",
                robot.colorSensor.getSaturation(), robot.colorSensor.getValue());
    }
}
