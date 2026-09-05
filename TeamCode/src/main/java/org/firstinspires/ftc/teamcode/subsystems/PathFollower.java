package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * The slice of Pedro's {@code Follower} that {@link Drivetrain} actually drives.
 *
 * <p>Exists so the drivetrain's command logic can run on the JVM against a fake. Pedro's
 * {@code Follower} needs real motors and a localizer to construct, which put every path command,
 * the hold semantics and the heading hold beyond the reach of a unit test. Everything the
 * drivetrain calls goes through this interface; {@link PedroPathFollower} is the production
 * implementation, and callers that need Pedro's full API (path building, error readouts) still get
 * the raw follower from {@link Drivetrain#getFollower()}.
 *
 * <p>The contract mirrors Pedro exactly, including its surprises: {@link #isBusy()} is false while
 * holding a pose, and {@link #startTeleopDrive()} both breaks following and ticks the follower.
 */
public interface PathFollower {
    /** Advances the follower one loop. */
    void update();

    Pose getPose();

    /** Corrects the live estimate. */
    void setPose(Pose pose);

    /** Establishes the localizer origin. */
    void setStartingPose(Pose pose);

    /** Leaves path mode and accepts stick vectors. Ticks the follower once. */
    void startTeleopDrive();

    void setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric);

    /** Starts following. {@code isBusy()} becomes true synchronously. */
    void followPath(PathChain path, boolean holdEnd);

    /** True while actively following. False while merely holding a pose. */
    boolean isBusy();

    /** Station-keeps at a pose. Clears {@code isBusy}. */
    void holdPoint(Pose pose);

    /** Rotates in place to an absolute heading. {@code isBusy()} is true until it arrives. */
    void turnTo(double headingRadians);

    /** True when within the given tolerances of a pose. Heading is not checked. */
    boolean atPose(Pose pose, double xToleranceInches, double yToleranceInches);

    void setMaxPower(double power);
}
