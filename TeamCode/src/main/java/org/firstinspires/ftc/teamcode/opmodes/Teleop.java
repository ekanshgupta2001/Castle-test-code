package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    private Robot robot;
    private boolean lastToggle = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.drivetrain.startTeleop();
    }

    @Override
    public void loop() {
        boolean toggle = gamepad1.options;
        if (toggle && !lastToggle) {
            robot.drivetrain.toggleFieldCentric();
        }
        lastToggle = toggle;

        robot.drivetrain.drive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        robot.update();

        telemetry.addData("Mode", robot.drivetrain.isFieldCentric() ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Pose", robot.drivetrain.getPose());
    }
}
