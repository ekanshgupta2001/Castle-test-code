package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;

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
public class Drivetrain {
    public static int DRIVER_CONTROL_PRIORITY = -1;

    private final Follower follower;
    private boolean fieldCentric = true;
    /** Tracks whether the follower is currently accepting stick vectors. See {@link #engageTeleop}. */
    private boolean teleopEngaged = false;

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
                    drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
                })
                .setDone(() -> false)
                .setEnd(ec -> drive(0, 0, 0))
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
        return Command.build()
                .setStart(() -> {
                    PathChain path = pathSupplier.get();
                    if (path != null) followPath(path, holdEnd);
                })
                .setDone(() -> !isFollowingPath())
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
