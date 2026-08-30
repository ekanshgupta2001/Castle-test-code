package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.DriveScaling;
import org.firstinspires.ftc.teamcode.util.MatchLogger;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.io.IOException;
import java.util.List;

/**
 * Main driver-controlled OpMode.
 *
 * <p>Driving is itself an Ivy command ({@code drivetrain.driverControlCommand}) rather than code in
 * {@code loop()}. That is what makes macros safe: scheduling one suspends driver control through the
 * scheduler, and ending or cancelling it restores driver control automatically. There is no
 * "am I in a macro?" flag for the loop to get wrong.
 */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    /** Stick deflection that counts as "the driver wants control back" and aborts a macro. */
    private static final double MACRO_ABORT_STICK = 0.25;

    private Robot robot;
    private MatchLogger logger;
    private String loggerError = null;
    private boolean localized = false;
    private boolean inheritedPose = false;

    private Command activeMacro = null;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();

        // Default commands. Both sit at priority -1 with SUSPEND, so any macro preempts them and
        // they resume by themselves when it finishes.
        robot.intake.defaultIdleCommand().schedule();
        robot.drivetrain.driverControlCommand(
                () -> DriveScaling.shape(-gamepad1.left_stick_y) * slowScale(),
                () -> DriveScaling.shape(-gamepad1.left_stick_x) * slowScale(),
                () -> DriveScaling.shape(-gamepad1.right_stick_x) * slowScale()
        ).schedule();

        try {
            logger = new MatchLogger("teleop");
        } catch (IOException e) {
            logger = null;
            loggerError = e.getMessage();
        }

        Drawing.init();

        // Inherit where autonomous left off. Without this, teleop starts with an unknown heading
        // while defaulting to field-centric drive - the mode that depends on heading most - so the
        // driver's first stick input sends the robot in an arbitrary direction.
        if (PoseStorage.hasPose()) {
            robot.drivetrain.setStartingPose(PoseStorage.getPose());
            robot.poseFusion.seed(PoseStorage.getPose());
            inheritedPose = true;
        }

        List<String> missing = robot.getMissingHardware();
        if (missing.isEmpty()) {
            telemetry.addLine("All hardware present.");
        } else {
            telemetry.addLine("MISSING HARDWARE (robot will still drive):");
            for (String m : missing) telemetry.addLine("  - " + m);
        }
        telemetry.update();
    }

    private double slowScale() {
        return DriveScaling.slowScale(gamepad1.left_trigger);
    }

    @Override
    public void init_loop() {
        robot.readSensors();
        // Re-evaluated every loop rather than latched: a reading going stale or a tag leaving view
        // should be visible to the drivers, not hidden behind a sticky "yes".
        localized = robot.tryLocalizeFromAprilTag();

        for (String m : robot.getMissingHardware()) telemetry.addLine("MISSING: " + m);
        telemetry.addData("Pose from auto?", inheritedPose ? "yes" : "no - drive is unreferenced");
        telemetry.addData("Localized?", localized ? "yes" : "looking for AprilTag...");
        telemetry.addData("Tags in view", robot.limelight.getBotposeTagCount());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addLine();
        telemetry.addLine("Driver: sticks=drive  options=field/robot  L-trig=slow  Y=reset heading");
        telemetry.addLine("        A=collect  X=servo-align  B=path-align  RB=relocalize  BACK=abort");
        telemetry.addLine("Operator: RB=intake  LB=outtake  B=eject  X=stop  Y=capture+hold");
    }

    @Override
    public void start() {
        // *WasPressed() latches until read, and init_loop() reads none of them. Without this, every
        // button bumped during init fires at once on the first loop tick - including A, which would
        // launch a vision macro the instant the match starts.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.drivetrain.startTeleop();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        robot.readSensors();      // 1. observe
        robot.updateLocalization();  //    blend any AprilTag fix into the pose estimate
        handleDriverInput();      // 2. decide
        handleOperatorInput();
        Scheduler.execute();      //    (driver control and macros both run here)
        robot.writeActuators();   // 3. act

        if (logger != null) logger.logRow(robot);
        updateTelemetry();
        draw();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (robot != null) robot.stop();
        if (logger != null) logger.close();
    }

    private void handleDriverInput() {
        if (gamepad1.optionsWasPressed()) robot.drivetrain.toggleFieldCentric();

        if (gamepad1.yWasPressed()) {
            // Escape hatch when field-centric drive has drifted: treat the current facing as
            // heading zero. Without this a bad localisation makes the robot undrivable.
            robot.drivetrain.resetHeading();
        }

        // Two ways out of a macro: an explicit abort button, or simply grabbing the sticks.
        if (macroRunning() && (gamepad1.backWasPressed() || driverWantsControl())) {
            abortMacro();
        }

        if (!macroRunning()) {
            if (gamepad1.aWasPressed()) startMacro(robot.macros.collectPollen());
            else if (gamepad1.xWasPressed()) startMacro(robot.macros.servoAlignToPollen());
            else if (gamepad1.bWasPressed()) startMacro(robot.macros.alignToPollen());
            else if (gamepad1.rightBumperWasPressed()) startMacro(robot.macros.relocalize());
        }
    }

    private void startMacro(Command macro) {
        activeMacro = macro;
        macro.schedule();
    }

    private boolean macroRunning() {
        return activeMacro != null && Scheduler.isScheduled(activeMacro);
    }

    private boolean driverWantsControl() {
        return Math.abs(gamepad1.left_stick_y) > MACRO_ABORT_STICK
                || Math.abs(gamepad1.left_stick_x) > MACRO_ABORT_STICK
                || Math.abs(gamepad1.right_stick_x) > MACRO_ABORT_STICK;
    }

    private void abortMacro() {
        Scheduler.cancel(activeMacro);
        // Cancelling the command releases the drivetrain resource, but the follower drives itself
        // once handed a path - this is the call that actually stops the robot.
        robot.abortMacro();
        activeMacro = null;
    }

    private void handleOperatorInput() {
        if (gamepad2.rightBumperWasPressed()) robot.intake.intakeCommand().schedule();
        if (gamepad2.leftBumperWasPressed())  robot.intake.outtakeCommand().schedule();
        if (gamepad2.bWasPressed())           robot.intake.ejectCommand().schedule();
        if (gamepad2.xWasPressed())           robot.intake.stopCommand().schedule();
        if (gamepad2.yWasPressed())           robot.intake.captureAndHoldCommand().schedule();
    }

    private void draw() {
        Drawing.drawRobot(robot.drivetrain.getPose());
        if (robot.limelight.hasStablePollen()) {
            Drawing.drawTarget(robot.limelight.estimatePollenFieldPose(robot.drivetrain.getPose()));
        }
        Drawing.sendPacket();
    }

    private void updateTelemetry() {
        telemetry.addData("Loop ms", "%.1f", loopMs);
        telemetry.addData("Battery V", "%.2f", batteryVolts());
        telemetry.addData("Macro", robot.macros.getStatus());
        if (loggerError != null) telemetry.addData("Logger FAILED", loggerError);
        telemetry.addLine();
        telemetry.addData("Drive", robot.drivetrain.isFieldCentric() ? "Field" : "Robot");
        telemetry.addData("Slow scale", "%.2f", slowScale());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Following path?", robot.drivetrain.isFollowingPath());
        telemetry.addLine();
        telemetry.addData("Intake mode", robot.intake.getMode());
        telemetry.addData("Intake target v", "%.0f", robot.intake.getTargetVelocity());
        telemetry.addData("Intake actual v", "%.0f", robot.intake.getVelocityTicksPerSec());
        telemetry.addData("Intake amps", "%.2f", robot.intake.getCurrentAmps());
        telemetry.addData("hasPollen?", robot.intake.hasPollen());
        telemetry.addData("Unjamming?", robot.intake.isUnjamming());
        telemetry.addLine();
        telemetry.addData("Localization", robot.poseFusion.getStatus());
        telemetry.addData("Pipeline", robot.macros.getPipelineName());
        telemetry.addData("LL tags", robot.limelight.getBotposeTagCount());
        telemetry.addData("Pollen lock?", robot.limelight.hasStablePollen());
        telemetry.addData("Pollen spread", "%.1f deg", robot.limelight.getPollenSpreadDegrees());
        telemetry.addData("Pollen range", "%.1f in", robot.limelight.estimatePollenDistanceInches());
        telemetry.addData("Color hue", "%.0f", robot.colorSensor.getHue());
        telemetry.addData("Color sat/val", "%.2f / %.2f",
                robot.colorSensor.getSaturation(), robot.colorSensor.getValue());
    }

    private double batteryVolts() {
        double lowest = Double.MAX_VALUE;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double v = s.getVoltage();
            if (v > 0 && v < lowest) lowest = v;
        }
        return lowest == Double.MAX_VALUE ? 0 : lowest;
    }
}
