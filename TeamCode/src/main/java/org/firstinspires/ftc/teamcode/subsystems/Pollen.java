package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.Collections;
import java.util.List;

public class Pollen {
    public static final int DEFAULT_PIPELINE = 1;

    // Mount geometry — tune to your robot before using estimateFieldPose()
    public static double CAMERA_HEIGHT_INCHES = 12.0;
    public static double CAMERA_PITCH_DEGREES = 20.0;          // positive = tilted toward floor
    public static double CAMERA_FORWARD_OFFSET_INCHES = 6.0;   // camera in front of robot center
    public static double POLLEN_HEIGHT_INCHES = 1.5;

    private final Limelight limelight;
    private final int pipelineIndex;
    private List<LLResultTypes.ColorResult> detections = Collections.emptyList();

    public Pollen(Limelight limelight) {
        this(limelight, DEFAULT_PIPELINE);
    }

    public Pollen(Limelight limelight, int pipelineIndex) {
        this.limelight = limelight;
        this.pipelineIndex = pipelineIndex;
    }

    public void activate() {
        limelight.switchPipeline(pipelineIndex);
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            detections = Collections.emptyList();
            return;
        }
        List<LLResultTypes.ColorResult> colors = result.getColorResults();
        detections = colors == null ? Collections.<LLResultTypes.ColorResult>emptyList() : colors;
    }

    public boolean hasPollen() {
        return !detections.isEmpty();
    }

    public int getCount() {
        return detections.size();
    }

    public List<LLResultTypes.ColorResult> getDetections() {
        return Collections.unmodifiableList(detections);
    }

    public LLResultTypes.ColorResult getPrimary() {
        return detections.isEmpty() ? null : detections.get(0);
    }

    public double getPrimaryTx() {
        return detections.isEmpty() ? 0 : detections.get(0).getTargetXDegrees();
    }

    public double getPrimaryTy() {
        return detections.isEmpty() ? 0 : detections.get(0).getTargetYDegrees();
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public double estimateDistanceInches() {
        if (detections.isEmpty()) return 0;
        double angleDownRad = Math.toRadians(CAMERA_PITCH_DEGREES - getPrimaryTy());
        double heightDiff = CAMERA_HEIGHT_INCHES - POLLEN_HEIGHT_INCHES;
        double tan = Math.tan(angleDownRad);
        return tan == 0 ? 0 : heightDiff / tan;
    }

    public Pose estimateFieldPose(Pose robotPose) {
        if (detections.isEmpty()) return robotPose;

        double distance = estimateDistanceInches();
        double txRad = Math.toRadians(getPrimaryTx());

        // Robot frame: +forward, +left. Limelight tx>0 means target right of crosshair → -left.
        double forward = distance * Math.cos(txRad) + CAMERA_FORWARD_OFFSET_INCHES;
        double left = -distance * Math.sin(txRad);

        double cosH = Math.cos(robotPose.getHeading());
        double sinH = Math.sin(robotPose.getHeading());
        double dx = forward * cosH - left * sinH;
        double dy = forward * sinH + left * cosH;

        double approachHeading = Math.atan2(dy, dx);
        return new Pose(robotPose.getX() + dx, robotPose.getY() + dy, approachHeading);
    }
}
