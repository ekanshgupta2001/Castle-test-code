package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {
    private final Follower follower;
    private boolean fieldCentric = true;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public void startTeleop() {
        follower.startTeleopDrive();
    }

    public void drive(double forward, double strafe, double turn) {
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

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public void update() {
        follower.update();
    }
}
