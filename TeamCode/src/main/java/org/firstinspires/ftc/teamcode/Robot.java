package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Pollen;

public class Robot {
    public static double POLLEN_COLOR_VALUE_THRESHOLD = 0.3;
    public static long POLLEN_PIPELINE_WARMUP_MS = 250;
    public static int APRILTAG_PIPELINE = 0;
    public static long LOCALIZATION_TIMEOUT_MS = 1500;

    public final Drivetrain drivetrain;
    public final Limelight limelight;
    public final Pollen pollen;
    public final ColorSensor colorSensor;
    public final Intake intake;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        limelight = new Limelight(hardwareMap);
        pollen = new Pollen(limelight);
        colorSensor = new ColorSensor(hardwareMap);
        intake = new Intake(hardwareMap);
        intake.setCapturedSupplier(this::pollenAtColorSensor);
    }

    public void update() {
        drivetrain.update();
        limelight.update();
        pollen.update();
        colorSensor.update();
        intake.update();
    }

    public Command collectPollen() {
        return sequential(
                instant(pollen::activate),
                waitMs(POLLEN_PIPELINE_WARMUP_MS),
                waitUntil(pollen::hasPollen),
                instant(this::startPathToPollen),
                deadline(
                        waitUntil(() -> !drivetrain.getFollower().isBusy()),
                        intake.captureAndHoldCommand()
                )
        );
    }

    private void startPathToPollen() {
        Pose current = drivetrain.getPose();
        Pose target = pollen.estimateFieldPose(current);

        drivetrain.getFollower().followPath(
                drivetrain.getFollower().pathBuilder()
                        .addPath(new BezierLine(current, target))
                        .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                        .build()
        );
    }

    private boolean pollenAtColorSensor() {
        return colorSensor.getValue() > POLLEN_COLOR_VALUE_THRESHOLD;
    }

    public boolean tryLocalizeFromAprilTag() {
        if (limelight.getPipelineIndex() != APRILTAG_PIPELINE) {
            limelight.switchPipeline(APRILTAG_PIPELINE);
        }
        limelight.update();
        Pose botpose = limelight.getBotposeAsPedroPose();
        if (botpose == null) return false;
        drivetrain.setPose(botpose);
        return true;
    }

    public Command localizeFromAprilTagCommand() {
        return sequential(
                instant(() -> limelight.switchPipeline(APRILTAG_PIPELINE)),
                race(
                        waitUntil(() -> limelight.getBotposeAsPedroPose() != null),
                        waitMs(LOCALIZATION_TIMEOUT_MS)
                ),
                instant(() -> {
                    Pose bp = limelight.getBotposeAsPedroPose();
                    if (bp != null) drivetrain.setPose(bp);
                })
        );
    }
}
