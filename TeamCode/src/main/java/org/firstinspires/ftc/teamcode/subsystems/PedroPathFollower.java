package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/** {@link PathFollower} backed by a real Pedro {@link Follower}. Pure delegation. */
public final class PedroPathFollower implements PathFollower {
    private final Follower follower;

    public PedroPathFollower(Follower follower) {
        this.follower = follower;
    }

    /** The wrapped follower, for callers that need Pedro's full API. */
    public Follower raw() {
        return follower;
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public Pose getPose() {
        return follower.getPose();
    }

    @Override
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    @Override
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    @Override
    public void startTeleopDrive() {
        follower.startTeleopDrive();
    }

    @Override
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    @Override
    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void holdPoint(Pose pose) {
        follower.holdPoint(pose);
    }

    @Override
    public void turnTo(double headingRadians) {
        follower.turnTo(headingRadians);
    }

    @Override
    public boolean atPose(Pose pose, double xToleranceInches, double yToleranceInches) {
        return follower.atPose(pose, xToleranceInches, yToleranceInches);
    }

    @Override
    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }
}
