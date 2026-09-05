package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * A {@link PathFollower} with Pedro's state machine and none of its motion.
 *
 * <p>Mirrors the real follower's surprises exactly: {@link #isBusy()} goes false when a hold
 * begins, {@link #startTeleopDrive()} breaks any hold, and {@link #followPath} makes it busy
 * synchronously. The test decides when a path "finishes" by calling {@link #finishPath()}.
 */
public final class FakePathFollower implements PathFollower {
    public Pose pose = new Pose(0, 0, 0);

    public boolean busy = false;
    public boolean holding = false;
    public boolean teleop = false;

    public PathChain lastPath = null;
    public boolean lastHoldEnd = false;
    public Double lastTurnTarget = null;
    public double lastForward = 0, lastStrafe = 0, lastTurn = 0;
    public boolean lastRobotCentric = false;

    public int updateCalls = 0;
    public int startTeleopDriveCalls = 0;
    public int followPathCalls = 0;
    public int setPoseCalls = 0;

    /** Ends the current path the way Pedro does: hold if asked, otherwise just stop being busy. */
    public void finishPath() {
        busy = false;
        if (lastHoldEnd) holding = true;
    }

    /** Ends a turnTo: no longer busy, and holding the new heading like Pedro's holdPoint. */
    public void finishTurn() {
        busy = false;
        holding = true;
        if (lastTurnTarget != null) pose = new Pose(pose.getX(), pose.getY(), lastTurnTarget);
    }

    @Override
    public void update() {
        updateCalls++;
    }

    @Override
    public Pose getPose() {
        return pose;
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
        setPoseCalls++;
    }

    @Override
    public void setStartingPose(Pose pose) {
        this.pose = pose;
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDriveCalls++;
        updateCalls++;          // Pedro ticks the follower inside startTeleopDrive()
        busy = false;
        holding = false;
        teleop = true;
    }

    @Override
    public void setTeleOpDrive(double forward, double strafe, double turn, boolean robotCentric) {
        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;
        lastRobotCentric = robotCentric;
    }

    @Override
    public void followPath(PathChain path, boolean holdEnd) {
        followPathCalls++;
        lastPath = path;
        lastHoldEnd = holdEnd;
        busy = true;
        holding = false;
        teleop = false;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    @Override
    public void holdPoint(Pose pose) {
        holding = true;
        busy = false;
        teleop = false;
    }

    @Override
    public void turnTo(double headingRadians) {
        lastTurnTarget = headingRadians;
        busy = true;
        holding = false;
        teleop = false;
    }

    @Override
    public boolean atPose(Pose target, double xTol, double yTol) {
        return Math.abs(target.getX() - pose.getX()) < xTol
                && Math.abs(target.getY() - pose.getY()) < yTol;
    }

    @Override
    public void setMaxPower(double power) {}
}
