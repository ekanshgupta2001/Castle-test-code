package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.math.Angles;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Wraps the Pedro follower: teleop driving, pose access, and path control.
 *
 * <p>Two Pedro modes share this one object. In <b>teleop mode</b> the follower consumes stick
 * vectors; in <b>path mode</b> it drives itself along a {@link PathChain} and ignores the sticks.
 * {@link #startTeleop()} is what switches back, and it must be called after a path ends — see
 * {@link #isFollowingPath()}.
 *
 * <h2>Arbitration</h2>
 *
 * This subsystem is an Ivy <em>resource</em>: every command that moves the robot declares
 * {@code requiring(drivetrain)}. {@link #driverControlCommand} sits at priority -1 with
 * {@link InterruptedBehavior#SUSPEND}, so scheduling any macro automatically parks driver control
 * and ending the macro automatically restores it. That is the whole cancellation story — there is
 * no flag to remember to clear, and no way for two commands to fight over the motors.
 *
 * <h2>Testability</h2>
 *
 * All motion goes through {@link PathFollower}, a thin interface over the follower, and all time
 * comes from a {@link Clock}. Both are injectable, so every command below runs on the JVM against
 * a fake follower and a fake clock. Callers that need Pedro's full API (path building, error
 * readouts for the logger) use {@link #getFollower()}, which is null when the drivetrain was built
 * on anything other than a real follower.
 */
@Configurable
public class Drivetrain {
    public static int DRIVER_CONTROL_PRIORITY = -1;

    /**
     * Whether the robot holds its heading when the driver is not turning.
     *
     * <p>Worth knowing what this depends on: heading hold is only as good as the localizer's
     * heading. If localisation has drifted, this will actively rotate the robot toward a heading
     * that no longer means what it should. That is what {@code Y} (reset heading) is for, and why
     * this can be switched off.
     */
    public static boolean HEADING_HOLD_ENABLED = true;
    /** Turn-stick magnitude above which the driver is considered to be steering. */
    public static double HEADING_HOLD_STICK_DEADBAND = 0.05;
    /** Error is in RADIANS here, so a gain of 1.5 maps 10 degrees (0.17 rad) to ~0.26 turn power. */
    public static double HEADING_HOLD_P = 1.5;
    public static double HEADING_HOLD_I = 0.0;
    public static double HEADING_HOLD_D = 0.08;
    public static double HEADING_HOLD_MAX_TURN = 0.4;
    /** Below this error, stop correcting — otherwise the robot hunts around the setpoint. */
    public static double HEADING_HOLD_TOLERANCE_RAD = Math.toRadians(1.0);

    /**
     * Minimum time a started path must run before it may report complete.
     *
     * <p>Guards against a path (or turn) that the follower considers finished on its very first
     * tick — a zero-length path, or a turn already within tolerance — reading as a perfect,
     * zero-time run. Same reasoning, and the same fix, as {@code PositionalMotor.MIN_MOVE_MS}.
     */
    public static long MIN_PATH_MS = 60;

    /** Null when the drivetrain could not be built; every motion call then no-ops. */
    private final PathFollower follower;
    /** The raw Pedro follower for callers needing its full API. Null under test. */
    private final Follower pedro;
    private final Clock clock;

    private boolean fieldCentric = true;
    /** Tracks whether the follower is currently accepting stick vectors. See {@link #engageTeleop}. */
    private boolean teleopEngaged = false;

    private final PIDFController headingController =
            new PIDFController(new PIDFCoefficients(HEADING_HOLD_P, HEADING_HOLD_I, HEADING_HOLD_D, 0));
    /** The heading being held, in radians, or null when the driver is steering. */
    private Double heldHeading = null;

    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, Clock.system());
    }

    public Drivetrain(HardwareMap hardwareMap, Clock clock) {
        this.clock = clock;
        Follower built = null;
        try {
            built = Constants.createFollower(hardwareMap);
        } catch (RuntimeException e) {
            // createFollower() looks up four drive motors plus the localizer. Any missing name
            // used to throw straight out of Robot's constructor and kill the OpMode.
            Hardware.recordFailure("drivetrain", "createFollower failed: " + e.getMessage());
        }
        pedro = built;
        follower = built == null ? null : new PedroPathFollower(built);
    }

    /**
     * Builds on an already-constructed follower. Tests inject a fake here; a robot with a
     * differently-built Pedro follower can pass a {@link PedroPathFollower}.
     */
    public Drivetrain(PathFollower follower, Clock clock) {
        this.clock = clock;
        this.follower = follower;
        this.pedro = follower instanceof PedroPathFollower ? ((PedroPathFollower) follower).raw() : null;
    }

    /** False when the drivetrain could not be built. All motion calls then no-op. */
    public boolean isAvailable() {
        return follower != null;
    }

    public void startTeleop() {
        if (follower != null) follower.startTeleopDrive();
        teleopEngaged = true;
    }

    /**
     * Puts the follower back into teleop mode, but only if it is not already there.
     *
     * <p>Guarded because {@code startTeleopDrive()} internally calls {@code follower.update()}.
     * Calling it every loop would tick the follower twice per cycle and corrupt its velocity and
     * derivative estimates.
     */
    private void engageTeleop() {
        if (!teleopEngaged) startTeleop();
    }

    /**
     * Applies stick vectors. Pedro's fourth argument is {@code isRobotCentric}, which is the
     * inverse of our {@link #fieldCentric} flag — hence the negation.
     *
     * <p>Only stores the vectors; {@link #update()} is what actually drives the motors.
     */
    public void drive(double forward, double strafe, double turn) {
        if (follower == null) return;
        follower.setTeleOpDrive(forward, strafe, turn, !fieldCentric);
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * Sets the localizer's origin reference. Use at init, before moving.
     *
     * <p>Not interchangeable with {@link #setPose}: this establishes where the robot <em>started</em>,
     * whereas setPose applies a correction to the running estimate.
     */
    public void setStartingPose(Pose pose) {
        if (follower != null && pose != null) follower.setStartingPose(pose);
        releaseHeadingHold();
    }

    /**
     * Corrects the live pose estimate, e.g. from an AprilTag fix.
     *
     * <p>Also drops the held heading. The hold's setpoint was captured in the old heading frame;
     * keeping it after the frame changes makes the controller chase a number that no longer means
     * anything, and the robot rotates by the size of the correction.
     */
    public void setPose(Pose pose) {
        if (follower != null && pose != null) follower.setPose(pose);
        releaseHeadingHold();
    }

    public Pose getPose() {
        return follower == null ? null : follower.getPose();
    }

    /**
     * The raw Pedro follower, for path building and diagnostics, or {@code null} when the
     * drivetrain is unavailable or was built on a non-Pedro {@link PathFollower}.
     */
    public Follower getFollower() {
        return pedro;
    }

    public void followPath(PathChain path, boolean holdEnd) {
        if (follower == null || path == null) return;
        teleopEngaged = false;
        follower.followPath(path, holdEnd);
    }

    /**
     * True while a path is actively being followed. Stick input is ignored during this.
     *
     * <p>False while the follower is merely <em>holding</em> a pose after a path that asked for
     * {@code holdEnd}: Pedro clears {@code isBusy} the moment the hold begins. Driver control
     * treats that as "free to drive" and re-engages teleop, which releases the hold.
     */
    public boolean isFollowingPath() {
        return follower != null && follower.isBusy();
    }

    /** Whether the robot is within the given distance of a pose on each axis. False when unavailable. */
    public boolean atPose(Pose target, double xToleranceInches, double yToleranceInches) {
        return follower != null && target != null
                && follower.atPose(target, xToleranceInches, yToleranceInches);
    }

    /**
     * Abandons the current path immediately and hands control back to the driver.
     *
     * <p>This is the abort path: cancelling the Ivy command that started a path does not itself stop
     * the follower, because the follower is driving on its own. Without this call the robot keeps
     * going to its target with the sticks locked out.
     */
    public void cancelPath() {
        if (follower == null) return;
        // startTeleopDrive() breaks following itself. Going through startTeleop() keeps the
        // teleopEngaged flag truthful, so driver control does not call startTeleopDrive() a
        // second time on its next tick and double-tick the follower.
        startTeleop();
    }

    /**
     * Treats the robot's current facing as heading zero, keeping its x/y position.
     *
     * <p>The driver escape hatch for field-centric drive: if localisation has drifted, "forward"
     * on the stick stops matching forward on the field, and this re-aligns them without a restart.
     */
    public void resetHeading() {
        if (follower == null) return;
        Pose p = follower.getPose();
        if (p == null) return;
        // setPose() releases the heading hold. Without that, the hold would still be aiming at the
        // heading this call just discarded and would spin the robot back toward it.
        setPose(new Pose(p.getX(), p.getY(), 0));
    }

    public void setMaxPower(double power) {
        if (follower != null) follower.setMaxPower(power);
    }

    // ---- Heading hold ----

    /**
     * Replaces a centred turn stick with a correction back toward the held heading.
     *
     * <p>A mecanum robot does not track straight on its own: uneven friction, a knocked wheel, or
     * contact with another robot all rotate it, and without correction the driver spends the whole
     * match nudging the turn stick to stay pointed where they already were.
     *
     * <p><b>It must never fight the driver.</b> Any deliberate turn input hands control straight
     * back and re-captures the heading on release, so the robot holds wherever the driver left it
     * rather than snapping back to where they started turning.
     *
     * @param turn the driver's raw turn request
     * @return the turn value to actually command
     */
    private double applyHeadingHold(double turn) {
        if (!HEADING_HOLD_ENABLED || follower == null) {
            heldHeading = null;
            return turn;
        }

        if (Math.abs(turn) >= HEADING_HOLD_STICK_DEADBAND) {
            // Driver is steering. Drop the setpoint so it is re-captured when they let go.
            heldHeading = null;
            return turn;
        }

        Pose pose = follower.getPose();
        if (pose == null) {
            heldHeading = null;
            return turn;
        }

        if (heldHeading == null) {
            heldHeading = pose.getHeading();
            headingController.reset();
            headingController.setCoefficients(
                    new PIDFCoefficients(HEADING_HOLD_P, HEADING_HOLD_I, HEADING_HOLD_D, 0));
            return 0;
        }

        // Fed as an error rather than a position, so the controller never sees the raw angles and
        // the 0/2pi seam cannot produce a full-speed spin the short way round.
        double error = Angles.angleError(pose.getHeading(), heldHeading);
        if (Math.abs(error) <= HEADING_HOLD_TOLERANCE_RAD) return 0;

        headingController.updateError(error);
        double correction = headingController.run();
        return Math.max(-HEADING_HOLD_MAX_TURN, Math.min(HEADING_HOLD_MAX_TURN, correction));
    }

    /** Forgets the held heading, so the next centred-stick loop captures a fresh one. */
    public void releaseHeadingHold() {
        heldHeading = null;
    }

    public boolean isHeadingHoldActive() {
        return heldHeading != null;
    }

    /** The heading being held in radians, or NaN when not holding. */
    public double getHeldHeading() {
        return heldHeading == null ? Double.NaN : heldHeading;
    }

    public void update() {
        if (follower != null) follower.update();
    }

    // ---- Ivy commands ----

    /**
     * The default command: feeds stick values to the follower whenever nothing else owns the
     * drivetrain. Schedule once at OpMode init.
     *
     * <p>Priority -1 plus {@link InterruptedBehavior#SUSPEND} is what makes macros cancellable for
     * free — any priority-0 command requiring this subsystem preempts it, and it comes back on its
     * own afterwards. {@link BlockedBehavior#QUEUE} keeps it from being silently dropped if it is
     * ever scheduled while something else already holds the resource.
     *
     * <p>The behaviour lives in {@code setExecute} rather than {@code setStart} because the
     * Scheduler's resume path re-adds a suspended command without re-calling {@code start()}.
     */
    public Command driverControlCommand(DoubleSupplier forward, DoubleSupplier strafe,
                                        DoubleSupplier turn) {
        return Command.build()
                .setExecute(() -> {
                    if (isFollowingPath()) return;
                    engageTeleop();
                    drive(forward.getAsDouble(), strafe.getAsDouble(),
                            applyHeadingHold(turn.getAsDouble()));
                })
                .setDone(() -> false)
                .setEnd(ec -> {
                    releaseHeadingHold();
                    drive(0, 0, 0);
                })
                .setPriority(DRIVER_CONTROL_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }

    /**
     * Follows a pre-built path. Same guarantees as {@link #followLazyCommand}, for a chain whose
     * endpoints are fixed and known in advance.
     */
    public Command followPathCommand(PathChain path, boolean holdEnd) {
        if (follower == null || path == null) return finishedCommand();
        return followLazyCommand(() -> path, holdEnd);
    }

    /**
     * Follows a path that is not known until the command actually starts.
     *
     * <p>Necessary for anything vision-driven. A {@code PathChain} caches its endpoints the first
     * time it is initialised, so a chain built once and stored would keep driving to the target it
     * saw on the first run. Building inside the supplier gives a fresh path per invocation.
     *
     * <p>A supplier returning {@code null} — no valid target — yields a command that finishes
     * immediately rather than moving the robot somewhere arbitrary.
     *
     * <h3>What {@code holdEnd} actually does</h3>
     * Pedro clears {@code isBusy} the moment the end-of-path hold begins, so the command completes
     * then. With {@code holdEnd = true} the follower is <em>left station-keeping</em> at the target
     * when the command ends naturally; the hold is released by the next path, by
     * {@link #cancelPath()}, or, in teleop, the moment driver control resumes. With
     * {@code holdEnd = false}, or whenever the command is interrupted, {@code cancelPath()} runs
     * and control goes straight back to the driver. An earlier version cancelled on every end,
     * which made the parameter a no-op.
     */
    public Command followLazyCommand(Supplier<PathChain> pathSupplier, boolean holdEnd) {
        // Boxed so the lambdas below share one instance of each; the command may be built once and
        // run more than once, and setStart resets both.
        final boolean[] started = new boolean[1];
        final long[] startedAt = new long[1];

        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    PathChain path = pathSupplier.get();
                    started[0] = path != null && follower != null;
                    if (started[0]) followPath(path, holdEnd);
                })
                .setDone(() -> {
                    // No target: finish at once rather than sitting still for MIN_PATH_MS. This is
                    // the documented "supplier returned null" behaviour that Macros relies on.
                    if (!started[0]) return true;
                    if (clock.nowMs() - startedAt[0] < MIN_PATH_MS) return false;
                    return !isFollowingPath();
                })
                .setEnd(ec -> {
                    // A natural end while asked to hold leaves the follower holding. Everything
                    // else - interrupted, suspended, or a path that was not asked to hold - hands
                    // control back, because a follower left in path mode keeps driving itself.
                    if (!started[0]) return;
                    if (ec != EndCondition.NATURALLY || !holdEnd) cancelPath();
                })
                .requiring(this);
    }

    /**
     * Turns in place to an absolute field heading, then hands control back to the driver.
     *
     * <p>Pedro's {@code turnTo} station-keeps at the new heading and clears {@code isBusy} on
     * arrival; the {@code setEnd} then releases that hold, because a snap is something a driver
     * does mid-drive and wants the sticks back from immediately.
     */
    public Command turnToCommand(double headingRadians) {
        if (follower == null) return finishedCommand();
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    // Cleared at start, not at build: building a command must not change state.
                    teleopEngaged = false;
                    startedAt[0] = clock.nowMs();
                    follower.turnTo(headingRadians);
                })
                .setDone(() -> clock.nowMs() - startedAt[0] >= MIN_PATH_MS && !follower.isBusy())
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /**
     * Holds the current position against pushing until interrupted.
     *
     * <p>Deliberately never finishes on its own: "hold" means "until something else wants the
     * drivetrain". Pedro's own hold command completes as soon as the error is within tolerance,
     * which for a robot already sitting still is immediately.
     */
    public Command holdCommand() {
        if (follower == null) return finishedCommand();
        return Command.build()
                .setStart(() -> {
                    teleopEngaged = false;
                    Pose here = follower.getPose();
                    if (here != null) follower.holdPoint(here);
                })
                .setDone(() -> false)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /** A command that completes on its first tick. Returned when there is no drivetrain to move. */
    private static Command finishedCommand() {
        return Command.build().setDone(() -> true);
    }
}
