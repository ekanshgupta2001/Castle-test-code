package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {
    private final Limelight3A limelight;
    private LLResult latestResult;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, "limelight", 0);
    }

    public Limelight(HardwareMap hardwareMap, String name, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.pipelineSwitch(pipeline);
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
        limelight.pipelineSwitch(pipeline);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }
}
