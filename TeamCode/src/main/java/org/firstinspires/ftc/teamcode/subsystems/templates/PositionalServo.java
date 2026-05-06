package org.firstinspires.ftc.teamcode.subsystems.templates;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class PositionalServo {
    public static long DEFAULT_TRAVEL_MS = 300;

    private final Servo servo;
    private final Map<String, Double> presets = new HashMap<>();
    private long travelMs = DEFAULT_TRAVEL_MS;
    private double currentPos;

    public PositionalServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, 0);
    }

    public PositionalServo(HardwareMap hardwareMap, String name, double initialPosition) {
        servo = hardwareMap.get(Servo.class, name);
        currentPos = initialPosition;
        servo.setPosition(initialPosition);
    }

    public PositionalServo preset(String name, double position) {
        presets.put(name, position);
        return this;
    }

    public void setTravelMs(long ms) {
        this.travelMs = ms;
    }

    public void setNow(double position) {
        currentPos = position;
        servo.setPosition(position);
    }

    public void setNow(String preset) {
        Double pos = presets.get(preset);
        if (pos == null) throw new IllegalArgumentException("unknown preset: " + preset);
        setNow(pos);
    }

    public double getPosition() {
        return currentPos;
    }

    public Command goToCommand(double position) {
        return sequential(
                instant(() -> setNow(position)).requiring(this),
                waitMs(travelMs)
        );
    }

    public Command goToCommand(String preset) {
        return sequential(
                instant(() -> setNow(preset)).requiring(this),
                waitMs(travelMs)
        );
    }
}
