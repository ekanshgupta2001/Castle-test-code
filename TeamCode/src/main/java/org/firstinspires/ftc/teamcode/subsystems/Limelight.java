package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {
    // Field is 144" square; Limelight botpose origin is field center, Pedro origin is corner.
    public static double FIELD_HALF_INCHES = 72.0;
    public static double BOTPOSE_HEADING_OFFSET_RAD = 0.0;

    private final Limelight3A limelight;
    private LLResult latestResult;
    private int currentPipeline;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight", 0);
    }

    public Limelight(HardwareMap hardwareMap, String name, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.pipelineSwitch(pipeline);
        currentPipeline = pipeline;
        limelight.start();
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTx() {
        return hasTarget() ? latestResult.getTx() : 0;
    }

    public double getTy() {
        return hasTarget() ? latestResult.getTy() : 0;
    }

    public double getTa() {
        return hasTarget() ? latestResult.getTa() : 0;
    }

    public void switchPipeline(int pipeline) {
        if (pipeline == currentPipeline) return;
        limelight.pipelineSwitch(pipeline);
        currentPipeline = pipeline;
    }

    public int getPipelineIndex() {
        return currentPipeline;
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public Pose3D getBotpose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    public Pose getBotposeAsPedroPose() {
        Pose3D bp = getBotpose();
        if (bp == null) return null;
        Position pos = bp.getPosition().toUnit(DistanceUnit.INCH);
        if (Double.isNaN(pos.x) || Double.isNaN(pos.y)) return null;
        YawPitchRollAngles ori = bp.getOrientation();
        double yaw = ori.getYaw(AngleUnit.RADIANS) + BOTPOSE_HEADING_OFFSET_RAD;
        return new Pose(pos.x + FIELD_HALF_INCHES, pos.y + FIELD_HALF_INCHES, yaw);
    }
}
