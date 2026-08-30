package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.VisionMath;

/**
 * One-button robot actions, composed from subsystem commands.
 *
 * <h2>Rules every macro here follows</h2>
 *
 * <ol>
 *   <li><b>Bounded.</b> Every wait is wrapped in {@code race(work, waitMs(timeout))}. An unbounded
 *       {@code waitUntil} holds its subsystems forever when its condition never becomes true, which
 *       silently disables the default commands for the rest of the match.</li>
 *   <li><b>Reports an outcome.</b> {@link #getOutcome()} says whether it worked, timed out, or never
 *       saw a target. A macro that fails silently is worse than one that does nothing.</li>
 *   <li><b>Declares its resources.</b> Requirements come from the composed subsystem commands, so
 *       Ivy suspends driver control while a macro runs and restores it afterwards.</li>
 *   <li><b>Builds vision paths lazily.</b> A {@code PathChain} caches its endpoints on first use, so
 *       a stored chain would drive to a stale target. See
 *       {@code Drivetrain.followLazyCommand}.</li>
 * </ol>
 *
 * <p>Note that a macro holds the drivetrain for its whole duration, including the search phase
 * before the robot moves. That is deliberate — it keeps arbitration simple — and it is safe because
 * the driver can abort at any point by touching a stick.
 *
 * <p>These are all game-agnostic: no field coordinates appear here. Scoring routines that depend on
 * the field layout belong in a separate class once that layout is known.
 */
@Configurable
public class Macros {
    public static long PIPELINE_WARMUP_MS = 250;
    public static long SEARCH_TIMEOUT_MS = 2000;
    public static long APPROACH_TIMEOUT_MS = 4000;
    public static long ALIGN_TIMEOUT_MS = 1500;
    public static long RELOCALIZE_TIMEOUT_MS = 1500;

    // Vision-servo gains. Error is in DEGREES and output is a turn command in [-1, 1], so P is
    // small by construction: 0.02 means a 10-degree error asks for 20% turn power.
    public static double ALIGN_P = 0.02;
    public static double ALIGN_I = 0.0;
    public static double ALIGN_D = 0.001;
    public static double ALIGN_MAX_TURN = 0.35;
    public static double ALIGN_TOLERANCE_DEGREES = 1.5;

    /** How the last macro finished. */
    public enum Outcome { IDLE, RUNNING, SUCCESS, TIMED_OUT, NO_TARGET, CANCELLED }

    private final Robot robot;
    private volatile Outcome outcome = Outcome.IDLE;
    private volatile String activeName = "idle";
    /** Whether the intake was already loaded when the current macro began. See {@link #collectPollen}. */
    private volatile boolean hadPollenAtStart = false;

    public Macros(Robot robot) {
        this.robot = robot;
    }

    public Outcome getOutcome() {
        return outcome;
    }

    public String getActiveName() {
        return activeName;
    }

    /** Human-readable one-liner for telemetry. */
    public String getStatus() {
        return activeName + " : " + outcome;
    }

    /** Call when an operator aborts a macro, so the reported outcome reflects what happened. */
    public void markCancelled() {
        if (outcome == Outcome.RUNNING) outcome = Outcome.CANCELLED;
        activeName = "idle";
    }

    private Command begin(String name) {
        return instant(() -> {
            activeName = name;
            outcome = Outcome.RUNNING;
        });
    }

    private Command finish(Outcome success, Outcome failure, java.util.function.BooleanSupplier ok) {
        return instant(() -> {
            outcome = ok.getAsBoolean() ? success : failure;
            activeName = "idle";
        });
    }

    // ---- Vision-driven macros ----

    /**
     * Switches to the pollen pipeline, waits for a stable detection, drives to it, and intakes.
     *
     * <p>Succeeds on <em>capture</em>, not merely on the path ending — a path that completes with an
     * empty intake is a failure, and reporting it as success hides a miss from the drivers.
     */
    public Command collectPollen() {
        return sequential(
                begin("collectPollen"),
                instant(() -> hadPollenAtStart = robot.intake.hasPollen()),
                instant(robot.limelight::activatePollenPipeline),
                waitMs(PIPELINE_WARMUP_MS),
                race(
                        waitUntil(robot.limelight::hasStablePollen),
                        waitMs(SEARCH_TIMEOUT_MS)
                ),
                race(
                        deadline(
                                robot.drivetrain.followLazyCommand(this::buildPollenPath, false),
                                robot.intake.captureAndHoldCommand()
                        ),
                        // Stop early on a NEW capture. Testing hasPollen() alone would end the race
                        // on tick one whenever the robot set off already carrying a piece, and would
                        // then report that stale piece as this macro's success.
                        waitUntil(this::capturedSomethingNew),
                        waitMs(APPROACH_TIMEOUT_MS)
                ),
                instant(robot.limelight::activateAprilTagPipeline),
                finish(Outcome.SUCCESS, Outcome.TIMED_OUT, this::capturedSomethingNew)
        );
    }

    private boolean capturedSomethingNew() {
        return robot.intake.hasPollen() && !hadPollenAtStart;
    }

    /**
     * Turns to face the detected blob without driving to it.
     *
     * <p>Cheap and low-risk compared to {@link #collectPollen()} — useful when the drivers want to
     * line up and take the last few inches themselves.
     */
    public Command alignToPollen() {
        return sequential(
                begin("alignToPollen"),
                instant(robot.limelight::activatePollenPipeline),
                waitMs(PIPELINE_WARMUP_MS),
                race(
                        waitUntil(robot.limelight::hasStablePollen),
                        waitMs(SEARCH_TIMEOUT_MS)
                ),
                race(
                        robot.drivetrain.followLazyCommand(this::buildTurnToPollenPath, true),
                        waitMs(ALIGN_TIMEOUT_MS)
                ),
                instant(robot.limelight::activateAprilTagPipeline),
                finish(Outcome.SUCCESS, Outcome.NO_TARGET, robot.limelight::hasStablePollen)
        );
    }

    /**
     * Closed-loop turn that drives the camera's {@code tx} to zero.
     *
     * <p>Different tool from {@link #alignToPollen()}, which plans a path to a computed heading and
     * is therefore only as accurate as the pose estimate and the mount calibration. This servos on
     * the raw camera error, so it converges on the target regardless of either — the right choice
     * for the last few degrees, and a good demonstration of feedback control on a real sensor.
     *
     * <p>Uses Pedro's {@link PIDFController} rather than a hand-rolled loop, so it behaves like the
     * rest of the robot's control and its gains are tuned the same way.
     */
    public Command servoAlignToPollen() {
        final PIDFController controller =
                new PIDFController(new PIDFCoefficients(ALIGN_P, ALIGN_I, ALIGN_D, 0));
        return sequential(
                begin("servoAlign"),
                instant(robot.limelight::activatePollenPipeline),
                waitMs(PIPELINE_WARMUP_MS),
                race(
                        waitUntil(robot.limelight::hasStablePollen),
                        waitMs(SEARCH_TIMEOUT_MS)
                ),
                race(
                        Command.build()
                                .setStart(() -> {
                                    controller.reset();
                                    controller.setTargetPosition(0);
                                    robot.drivetrain.startTeleop();
                                })
                                .setExecute(() -> {
                                    if (!robot.limelight.hasStablePollen()) return;
                                    // tx > 0 means the target is right of centre, so we must turn
                                    // right, which is a negative (clockwise) turn command.
                                    controller.updatePosition(robot.limelight.getFilteredPollenTx());
                                    double turn = -clamp(controller.run(), ALIGN_MAX_TURN);
                                    robot.drivetrain.drive(0, 0, turn);
                                })
                                .setDone(() -> robot.limelight.hasStablePollen()
                                        && Math.abs(robot.limelight.getFilteredPollenTx())
                                           <= ALIGN_TOLERANCE_DEGREES)
                                .setEnd(ec -> robot.drivetrain.drive(0, 0, 0))
                                .requiring(robot.drivetrain),
                        waitMs(ALIGN_TIMEOUT_MS)
                ),
                instant(robot.limelight::activateAprilTagPipeline),
                finish(Outcome.SUCCESS, Outcome.TIMED_OUT,
                        () -> robot.limelight.hasStablePollen()
                                && Math.abs(robot.limelight.getFilteredPollenTx())
                                   <= ALIGN_TOLERANCE_DEGREES)
        );
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    /** Switches to AprilTags and corrects the pose estimate, if a trustworthy fix is available. */
    public Command relocalize() {
        return sequential(
                begin("relocalize"),
                instant(robot.limelight::activateAprilTagPipeline),
                waitMs(PIPELINE_WARMUP_MS),
                race(
                        waitUntil(() -> robot.limelight.getBotposeAsPedroPose() != null),
                        waitMs(RELOCALIZE_TIMEOUT_MS)
                ),
                instant(robot::tryLocalizeFromAprilTag),
                finish(Outcome.SUCCESS, Outcome.NO_TARGET,
                        () -> robot.limelight.getBotposeAsPedroPose() != null)
        );
    }

    // ---- Path construction ----

    /**
     * Builds a fresh straight-line path to the current pollen estimate, or {@code null} if there
     * isn't a trustworthy one. Called at command start, never stored.
     */
    private PathChain buildPollenPath() {
        Pose current = robot.drivetrain.getPose();
        if (current == null || !robot.limelight.hasStablePollen()) return null;

        Pose target = robot.limelight.estimatePollenFieldPose(current);
        if (target == null) return null;

        return robot.drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();
    }

    /**
     * A zero-translation path that only rotates to face the blob.
     *
     * <p>Uses a path rather than {@code turnTo} so the heading interpolation and completion
     * tolerance come from the same tuned constants as every other motion.
     */
    private PathChain buildTurnToPollenPath() {
        Pose current = robot.drivetrain.getPose();
        if (current == null || !robot.limelight.hasStablePollen()) return null;

        double[] robotFrame = robot.limelight.estimatePollenInRobotFrame();
        if (robotFrame == null) return null;

        double heading = VisionMath.headingToward(
                current.getHeading(), robotFrame[0], robotFrame[1]);
        Pose target = new Pose(current.getX(), current.getY(), heading);

        return robot.drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), heading)
                .build();
    }

    /** Convenience for telemetry: which pipeline the camera is on right now. */
    public String getPipelineName() {
        int p = robot.limelight.getPipelineIndex();
        if (p == Limelight.APRILTAG_PIPELINE_INDEX) return "apriltag";
        if (p == Limelight.POLLEN_PIPELINE_INDEX) return "pollen";
        return "pipeline " + p;
    }
}
