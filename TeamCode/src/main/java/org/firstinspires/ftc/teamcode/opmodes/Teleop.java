package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.DriveScaling;
import org.firstinspires.ftc.teamcode.util.MatchLogger;

import java.io.IOException;

@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    private Robot robot;
    private MatchLogger logger;
    private boolean wasFollowingPath = false;
    private boolean localizedAtInit = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        robot.intake.defaultIdleCommand().schedule();
        try {
            logger = new MatchLogger("teleop");
        } catch (IOException ignored) {
            logger = null;
        }
    }

    @Override
    public void init_loop() {
        robot.limelight.update();
        if (robot.tryLocalizeFromAprilTag()) {
            localizedAtInit = true;
        }
        telemetry.addData("Localized?", localizedAtInit ? "yes" : "looking for AprilTag...");
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addLine();
        telemetry.addLine("Driver: sticks=drive  options=field-centric  L-trig=slow  A=collectPollen");
        telemetry.addLine("Operator: RB=intake  LB=outtake  B=eject  X=stop  Y=capture+hold");
    }

    @Override
    public void start() {
        robot.drivetrain.startTeleop();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) robot.drivetrain.toggleFieldCentric();
        if (gamepad1.aWasPressed()) robot.collectPollen().schedule();

        if (gamepad2.rightBumperWasPressed()) robot.intake.intakeCommand().schedule();
        if (gamepad2.leftBumperWasPressed())  robot.intake.outtakeCommand().schedule();
        if (gamepad2.bWasPressed())           robot.intake.ejectCommand().schedule();
        if (gamepad2.xWasPressed())           robot.intake.stopCommand().schedule();
        if (gamepad2.yWasPressed())           robot.intake.captureAndHoldCommand().schedule();

        Scheduler.execute();

        boolean isFollowingPath = robot.drivetrain.getFollower().isBusy();
        if (!isFollowingPath) {
            if (wasFollowingPath) robot.drivetrain.startTeleop();

            double scale = DriveScaling.slowScale(gamepad1.left_trigger);
            double forward = DriveScaling.shape(-gamepad1.left_stick_y) * scale;
            double strafe  = DriveScaling.shape(-gamepad1.left_stick_x) * scale;
            double turn    = DriveScaling.shape(-gamepad1.right_stick_x) * scale;
            robot.drivetrain.drive(forward, strafe, turn);
        }
        wasFollowingPath = isFollowingPath;

        robot.update();

        if (logger != null) logger.logRow(robot);

        telemetry.addData("Drive", robot.drivetrain.isFieldCentric() ? "Field" : "Robot");
        telemetry.addData("Slow scale", "%.2f", DriveScaling.slowScale(gamepad1.left_trigger));
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Following path?", isFollowingPath);
        telemetry.addLine();
        telemetry.addData("Intake target v", "%.0f", robot.intake.getTargetVelocity());
        telemetry.addData("Intake actual v", "%.0f", robot.intake.getVelocityTicksPerSec());
        telemetry.addData("Intake amps", "%.2f", robot.intake.getCurrentAmps());
        telemetry.addData("hasPollen?", robot.intake.hasPollen());
        telemetry.addData("Stalled?", robot.intake.isStalled());
        telemetry.addLine();
        telemetry.addData("LL target?", robot.limelight.hasTarget());
        telemetry.addData("Color value", "%.3f", robot.colorSensor.getValue());
    }

    @Override
    public void stop() {
        if (logger != null) logger.close();
    }
}
