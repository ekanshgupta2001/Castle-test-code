package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Pre-match diagnostics. Reports what is wired, what is missing, and whether each subsystem
 * actually responds — then leaves every actuator stopped.
 */
@TeleOp(name = "SelfTest", group = "Diagnostics")
public class SelfTest extends LinearOpMode {
    private static final long SPINUP_MS = 600;
    private static final double INTAKE_PASS_FRACTION = 0.5;

    private final List<String> results = new ArrayList<>();
    private boolean allPass = true;

    @Override
    public void runOpMode() {
        // Results must survive across update() calls, otherwise each new line wipes the previous
        // one and the final summary points at a blank screen.
        telemetry.setAutoClear(false);
        Scheduler.reset();

        Robot robot;
        try {
            robot = new Robot(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("X Robot construction FAILED: " + e);
            telemetry.update();
            while (opModeInInit()) idle();
            return;
        }

        telemetry.addLine("Robot constructed.");
        List<String> missing = robot.getMissingHardware();
        if (missing.isEmpty()) {
            telemetry.addLine("OK  every configured device resolved");
        } else {
            telemetry.addLine("X  MISSING DEVICES - fix these names in the robot config:");
            for (String m : missing) telemetry.addLine("     - " + m);
        }
        telemetry.addLine();
        // readSensors() is what samples the battery, so take one reading before reporting it.
        robot.readSensors();
        telemetry.addData("Battery", "%.2f V", robot.getBatteryVolts());
        telemetry.addLine();
        telemetry.addLine("Press START to run active checks (the intake WILL spin).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            checkBattery(robot);
            checkDrivetrain(robot);
            checkLimelight(robot);
            checkColorSensor(robot);
            checkIntake(robot);
        } finally {
            // Whatever happened above, leave the hardware safe. Setting the field alone is not
            // enough - writeActuators() is what actually pushes the zero to the motor.
            robot.intake.stop();
            robot.writeActuators();
            robot.stop();
        }

        telemetry.clearAll();
        telemetry.addLine("===== SELF TEST RESULTS =====");
        for (String r : results) telemetry.addLine(r);
        telemetry.addLine();
        telemetry.addLine(allPass ? "ALL PASS" : "FAILURES PRESENT - see above");
        telemetry.addLine();
        telemetry.addLine("Push the robot by hand to verify odometry tracks:");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {
            robot.readSensors();
            robot.drivetrain.update();
            telemetry.addData("Pose", robot.drivetrain.getPose());
            telemetry.update();
            sleep(50);
        }
    }

    private void checkBattery(Robot robot) {
        double v = robot.getBatteryVolts();
        report("Battery", v >= 12.0, String.format("%.2f V (want >= 12.0)", v));
    }

    private void checkDrivetrain(Robot robot) {
        if (!robot.drivetrain.isAvailable()) {
            report("Drivetrain", false, "follower could not be built - check motor names");
            return;
        }
        Pose pose = robot.drivetrain.getPose();
        report("Drivetrain", pose != null, pose == null ? "pose null" : "follower live, pose=" + pose);
    }

    private void checkLimelight(Robot robot) {
        if (!robot.limelight.isAvailable()) {
            report("Limelight", false, "not in config");
            return;
        }
        LLStatus status = robot.limelight.getStatus();
        boolean pass = status != null && status.getFps() > 0;
        report("Limelight", pass, status == null
                ? "no status - camera not responding"
                : "name=" + status.getName() + " fps=" + (int) status.getFps()
                  + " pipeline=" + status.getPipelineIndex());
    }

    private void checkColorSensor(Robot robot) {
        if (!robot.colorSensor.isAvailable()) {
            report("ColorSensor", false, "not in config");
            return;
        }
        robot.readSensors();
        float sat = robot.colorSensor.getSaturation();
        float val = robot.colorSensor.getValue();
        // A totally dead sensor reads exactly zero on every channel every time.
        boolean pass = val > 0.001f || sat > 0.001f;
        report("ColorSensor", pass, String.format("hue=%.0f sat=%.2f val=%.2f%s",
                robot.colorSensor.getHue(), sat, val,
                robot.colorSensor.hasLight() ? " (light on)" : " (no switchable light)"));
    }

    private void checkIntake(Robot robot) {
        if (!robot.intake.isAvailable()) {
            report("Intake", false, "not in config");
            return;
        }
        double target = Intake.INTAKE_TICKS_PER_SEC;

        // Order matters: command it, PUSH that command to the motor, THEN let it spin up, then
        // sample. Sleeping before writeActuators() would sample a motor that was never told to move.
        robot.intake.intake();
        robot.writeActuators();
        sleep(SPINUP_MS);
        robot.readSensors();
        double forwardV = robot.intake.getVelocityTicksPerSec();

        robot.intake.stop();
        robot.writeActuators();
        sleep(200);

        boolean pass = forwardV > target * INTAKE_PASS_FRACTION;
        report("Intake", pass, String.format(
                "commanded %.0f, reached %.0f (%.0f%% of request; free-speed ceiling %.0f)",
                target, forwardV, 100 * forwardV / target, Intake.MOTOR_FREE_SPEED_TICKS_PER_SEC));
        if (forwardV > 0 && forwardV < target * 0.95) {
            results.add("     note: if this is pinned well below the request, INTAKE_TICKS_PER_SEC");
            results.add("     may exceed what the motor can physically do. See Intake javadoc.");
        }
    }

    private void report(String subsystem, boolean pass, String detail) {
        if (!pass) allPass = false;
        results.add((pass ? "OK  " : "X   ") + subsystem + " - " + detail);
        telemetry.addLine((pass ? "OK  " : "X   ") + subsystem + " - " + detail);
        telemetry.update();
    }
}
