package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class PositionalMotor {
    public static double DEFAULT_POWER = 0.7;
    public static int DEFAULT_TOLERANCE_TICKS = 15;

    private final DcMotorEx motor;
    private final Map<String, Integer> presets = new HashMap<>();
    private double maxPower = DEFAULT_POWER;
    private int toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private int target = 0;

    public PositionalMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, DcMotorSimple.Direction.FORWARD);
    }

    public PositionalMotor(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        motor.setPower(maxPower);
    }

    public PositionalMotor preset(String name, int ticks) {
        presets.put(name, ticks);
        return this;
    }

    public void setMaxPower(double power) {
        this.maxPower = power;
        motor.setPower(power);
    }

    public void setTolerance(int ticks) {
        this.toleranceTicks = ticks;
    }

    public void goToTicks(int ticks) {
        target = ticks;
        motor.setTargetPosition(ticks);
        motor.setPower(maxPower);
    }

    public void goToPreset(String name) {
        Integer ticks = presets.get(name);
        if (ticks == null) throw new IllegalArgumentException("unknown preset: " + name);
        goToTicks(ticks);
    }

    public int getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }

    public boolean atTarget() {
        return Math.abs(motor.getCurrentPosition() - target) <= toleranceTicks;
    }

    public void update() {
    }

    public Command goToCommand(int ticks) {
        return Command.build()
                .setStart(() -> goToTicks(ticks))
                .setDone(this::atTarget)
                .requiring(this);
    }

    public Command goToCommand(String preset) {
        return Command.build()
                .setStart(() -> goToPreset(preset))
                .setDone(this::atTarget)
                .requiring(this);
    }
}
