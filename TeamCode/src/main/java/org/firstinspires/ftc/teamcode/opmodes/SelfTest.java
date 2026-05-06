package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "SelfTest", group = "Diagnostics")
public class SelfTest extends LinearOpMode {

    private static final double INTAKE_VELOCITY_PASS_THRESHOLD = 500;
    private static final long SPIN_MS = 250;

    @Override
    public void runOpMode() {
        boolean robotOk;
        try {
            new Robot(hardwareMap);
            robotOk = true;
        } catch (Exception e) {
            telemetry.addLine("✗ Robot construction FAILED: " + e.getMessage());
            telemetry.update();
            return;
        }

        Robot robot = new Robot(hardwareMap);
        telemetry.addLine("✓ Robot constructed");
        telemetry.addLine("Press START to run subsystem checks.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean drivetrainPass = robot.drivetrain.getPose() != null;
        report("Drivetrain", drivetrainPass, robot.drivetrain.getPose() == null
                ? "pose null"
                : "pose=" + robot.drivetrain.getPose());

        robot.intake.intake();
        sleep(SPIN_MS);
        robot.update();
        double forwardV = robot.intake.getVelocityTicksPerSec();
        robot.intake.outtake();
        sleep(SPIN_MS);
        robot.update();
        double reverseV = robot.intake.getVelocityTicksPerSec();
        robot.intake.stop();
        boolean intakePass = forwardV > INTAKE_VELOCITY_PASS_THRESHOLD
                && reverseV < -INTAKE_VELOCITY_PASS_THRESHOLD;
        report("Intake", intakePass,
                String.format("fwd=%.0f rev=%.0f (need |v|>%.0f)",
                        forwardV, reverseV, INTAKE_VELOCITY_PASS_THRESHOLD));

        robot.limelight.update();
        LLStatus status = robot.limelight.getStatus();
        boolean limelightPass = status != null && status.getName() != null && status.getFps() > 0;
        report("Limelight", limelightPass, status == null
                ? "no status"
                : "name=" + status.getName() + " fps=" + (int) status.getFps());

        robot.colorSensor.update();
        double v1 = robot.colorSensor.getValue();
        sleep(100);
        robot.colorSensor.update();
        double v2 = robot.colorSensor.getValue();
        boolean colorPass = v1 >= 0 && v1 <= 1 && v2 >= 0 && v2 <= 1;
        report("ColorSensor", colorPass,
                String.format("v1=%.3f v2=%.3f", v1, v2));

        boolean allPass = drivetrainPass && intakePass && limelightPass && colorPass;
        telemetry.addLine();
        telemetry.addLine(allPass ? "✓✓✓ ALL PASS ✓✓✓" : "✗ FAILURES PRESENT — see above");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            sleep(100);
        }
    }

    private void report(String subsystem, boolean pass, String detail) {
        telemetry.addLine((pass ? "✓ " : "✗ ") + subsystem + " — " + detail);
        telemetry.update();
    }
}
