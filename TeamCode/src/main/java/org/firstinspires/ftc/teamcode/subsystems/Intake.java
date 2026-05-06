package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.BooleanSupplier;

public class Intake {
    // 435 rpm GoBilda 5202: 537.7 ticks/rev → ~3898 ticks/sec at free speed. 3800 leaves headroom.
    public static double INTAKE_TICKS_PER_SEC = 3800;
    public static double OUTTAKE_TICKS_PER_SEC = -2000;
    public static double EJECT_TICKS_PER_SEC = -3800;
    public static double HOLD_TICKS_PER_SEC = 200;

    public static double STALL_CURRENT_AMPS = 5.0;
    public static long STALL_TIMEOUT_MS = 200;
    public static double UNJAM_TICKS_PER_SEC = -3800;
    public static long UNJAM_DURATION_MS = 150;
    public static boolean ANTI_JAM_ENABLED = true;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    private final DcMotorEx motor;
    private double targetVelocity = 0;
    private boolean hasPollen = false;
    private BooleanSupplier capturedSupplier = () -> false;

    private long stallStartMs = 0;
    private long unjamUntilMs = 0;

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "IM");
    }

    public Intake(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setCapturedSupplier(BooleanSupplier supplier) {
        this.capturedSupplier = supplier == null ? () -> false : supplier;
    }

    public void setVelocity(double ticksPerSec) {
        targetVelocity = ticksPerSec;
    }

    public void intake() {
        setVelocity(INTAKE_TICKS_PER_SEC);
    }

    public void outtake() {
        setVelocity(OUTTAKE_TICKS_PER_SEC);
        markEmpty();
    }

    public void eject() {
        setVelocity(EJECT_TICKS_PER_SEC);
        markEmpty();
    }

    public void hold() {
        setVelocity(HOLD_TICKS_PER_SEC);
    }

    public void stop() {
        setVelocity(0);
    }

    public boolean hasPollen() {
        return hasPollen;
    }

    public void markCaptured() {
        hasPollen = true;
    }

    public void markEmpty() {
        hasPollen = false;
    }

    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getVelocityTicksPerSec() {
        return motor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isStalled() {
        return stallStartMs != 0;
    }

    public boolean isUnjamming() {
        return unjamUntilMs > System.currentTimeMillis();
    }

    public void update() {
        long now = System.currentTimeMillis();

        if (targetVelocity > 0 && !hasPollen && capturedSupplier.getAsBoolean()) {
            hasPollen = true;
        }

        if (ANTI_JAM_ENABLED && targetVelocity > 0) {
            if (isUnjamming()) {
                motor.setVelocity(UNJAM_TICKS_PER_SEC);
                return;
            }
            if (motor.getCurrent(CurrentUnit.AMPS) > STALL_CURRENT_AMPS) {
                if (stallStartMs == 0) {
                    stallStartMs = now;
                } else if (now - stallStartMs >= STALL_TIMEOUT_MS) {
                    unjamUntilMs = now + UNJAM_DURATION_MS;
                    stallStartMs = 0;
                    motor.setVelocity(UNJAM_TICKS_PER_SEC);
                    return;
                }
            } else {
                stallStartMs = 0;
            }
        } else {
            stallStartMs = 0;
        }

        motor.setVelocity(targetVelocity);
    }

    public Command intakeCommand() {
        return Command.build()
                .setStart(this::intake)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command outtakeCommand() {
        return Command.build()
                .setStart(this::outtake)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command ejectCommand() {
        return Command.build()
                .setStart(this::eject)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command holdCommand() {
        return Command.build()
                .setStart(this::hold)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command stopCommand() {
        return Command.build()
                .setStart(this::stop)
                .setDone(() -> true)
                .requiring(this);
    }

    public Command runForMs(double ticksPerSec, long ms) {
        Command run = Command.build()
                .setStart(() -> setVelocity(ticksPerSec))
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
        return race(run, waitMs(ms));
    }

    public Command captureCommand() {
        return captureCommand(capturedSupplier);
    }

    public Command captureCommand(BooleanSupplier captured) {
        return Command.build()
                .setStart(this::intake)
                .setDone(captured)
                .setEnd(ec -> {
                    if (ec == EndCondition.NATURALLY) markCaptured();
                    stop();
                })
                .requiring(this);
    }

    public Command captureAndHoldCommand() {
        return captureAndHoldCommand(capturedSupplier);
    }

    public Command captureAndHoldCommand(BooleanSupplier captured) {
        return sequential(captureCommand(captured), holdCommand());
    }

    // Schedule once at OpMode init. Suspends when a real intake command takes
    // the resource and resumes when that command ends. Holds gently when carrying
    // pollen, otherwise idle.
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(() -> {
                    if (hasPollen) hold();
                    else stop();
                })
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .requiring(this);
    }
}
