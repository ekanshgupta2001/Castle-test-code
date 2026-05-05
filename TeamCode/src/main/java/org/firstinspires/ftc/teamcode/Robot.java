package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class Robot {
    public final Drivetrain drivetrain;
    public final Limelight limelight;
    public final ColorSensor colorSensor;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        limelight = new Limelight(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
    }

    public void update() {
        drivetrain.update();
        limelight.update();
        colorSensor.update();
    }
}
