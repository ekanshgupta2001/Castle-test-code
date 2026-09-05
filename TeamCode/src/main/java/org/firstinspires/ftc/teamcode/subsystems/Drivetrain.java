package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Wraps the Pedro {@link Follower}: teleop driving, pose access, and path control.
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
     * <p>{@code follower.isBusy()} is not guaranteed to be true on the same tick that
     * {@code followPath()} was called. Without this guard a path command can satisfy
     * {@code !isFollowingPath()} on its very first {@code done()} check and finish instantly —
     * which looks like a path that ran perfectly and took zero time. Same reasoning, and the same
     * fix, as {@code PositionalMotor.MIN_MOVE_MS}.
     */
    public static long MIN_PATH_MS = 60;

    private final Follower follower;
    private boolean fieldCentric = true;
    /** Tracks whether the follower is currently accepting stick vectors. See {@link #engageTeleop}. */
    private boolean teleopEngaged = false;

    private final PIDFController headingController =
            new PIDFController(new PIDFCoefficients(HEADING_HOLD_P, HEADING_HOLD_I, HEADING_HOLD_D, 0));
    /** The heading being held, in radians, or null when the driver is steering. */
    private Double heldHeading = null;

    public Drivetrain(HardwareMap hardwareMap) {
        Follower built = null;
        try {
            built = Constants.createFollower(hardwareMap);
        } catch (RuntimeException e) {
            // createFollower() looks up four drive motors plus the localizer. Any missing name
            // used to throw straight out of Robot's constructor and kill the OpMode.
            Hardware.recordFailure("drivetrain", "createFollower failed: " + e.getMessage());
        }
        follower = built;
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
    }

    /** Corrects the live pose estimate, e.g. from an AprilTag fix. */
    public void setPose(Pose pose) {
        if (follower != null && pose != null) follower.setPose(pose);
    }

    public Pose getPose() {
        return follower == null ? null : follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public void followPath(PathChain path, boolean holdEnd) {
        if (follower == null || path == null) return;
        teleopEngaged = false;
        follower.followPath(path, holdEnd);
    }

    /** True while a path is actively being followed. Stick input is ignored during this. */
    public boolean isFollowingPath() {
        return follower != null && follower.isBusy();
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
        follower.breakFollowing();
        follower.startTeleopDrive();
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
        follower.setPose(new Pose(p.getX(), p.getY(), 0));
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
     * Follows a pre-built path.
     *
     * <p>The {@code setEnd} is the important part: an interrupted command must also stop the
     * follower, because once handed a path the follower drives itself and would otherwise carry on
     * to its target with the sticks locked out.
     */
    public Command followPathCommand(PathChain path, boolean holdEnd) {
        if (follower == null || path == null) return Command.build().setDone(() -> true);
        teleopEngaged = false;
        return PedroCommands.follow(follower, path, holdEnd)
                .setEnd(ec -> cancelPath())
                .requiring(this);
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
     */
    public Command followLazyCommand(Supplier<PathChain> pathSupplier, boolean holdEnd) {
        // Boxed so the lambdas below share one instance of each; the command may be built once and
        // run more than once, and setStart resets both.
        final boolean[] started = new boolean[1];
        final long[] startedAt = new long[1];

        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    PathChain path = pathSupplier.get();
                    started[0] = path != null;
                    if (started[0]) followPath(path, holdEnd);
                })
                .setDone(() -> {
                    // No target: finish at once rather than sitting still for MIN_PATH_MS. This is
                    // the documented "supplier returned null" behaviour that Macros relies on.
                    if (!started[0]) return true;
                    if (System.currentTimeMillis() - startedAt[0] < MIN_PATH_MS) return false;
                    return !isFollowingPath();
                })
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    public Command turnToCommand(double headingRadians) {
        if (follower == null) return Command.build().setDone(() -> true);
        teleopEngaged = false;
        return PedroCommands.turnTo(follower, headingRadians)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /** Holds the current position against pushing. */
    public Command holdCommand() {
        if (follower == null) return Command.build().setDone(() -> true);
        teleopEngaged = false;
        return PedroCommands.hold(follower)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }
}
