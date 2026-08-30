package org.firstinspires.ftc.teamcode.subsystems.templates;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Worked example: a two-stage scoring mechanism built entirely from the templates.
 *
 * <p><b>This class is a reference, not part of the robot.</b> Nothing constructs it. Copy it into
 * {@code subsystems/} and adapt it when you build a real lift — that is what it is here for.
 *
 * <h2>What it demonstrates</h2>
 * <ol>
 *   <li>Enum-named positions, so {@code lift.goTo(Level.HIGH)} is checked by the compiler.</li>
 *   <li>Composing two mechanisms into one subsystem, with the subsystem exposing <em>intent</em>
 *       ({@code scoreHigh()}) rather than making callers sequence the parts themselves.</li>
 *   <li>Ordering that protects the hardware: the claw closes before the lift rises, and the lift
 *       returns to DOWN only after the claw has released. Encoding that here means no OpMode can
 *       get it wrong.</li>
 *   <li>{@code isAvailable()} propagation, so a missing motor degrades this subsystem instead of
 *       killing the OpMode.</li>
 * </ol>
 *
 * <h2>Wiring it into the robot</h2>
 * <pre>
 *   // in Robot's constructor
 *   lift = new ExampleLift(hardwareMap);
 *
 *   // in Robot.writeActuators()
 *   lift.update();
 *
 *   // from an OpMode
 *   if (gamepad2.dpadUpWasPressed()) lift.scoreHigh().schedule();
 * </pre>
 */
public class ExampleLift {

    /** Heights the lift can be commanded to. Add a level here and the presets below; that is all. */
    public enum Level { DOWN, LOW, HIGH }

    /** Claw positions. Two states, but still an enum — {@code goTo(OPEN)} beats {@code goTo(0.65)}. */
    public enum Grip { OPEN, CLOSED }

    // Encoder counts and servo positions are the most-tuned numbers on any real mechanism.
    // Measure them by jogging the hardware, then paste them here.
    public static int TICKS_DOWN = 0;
    public static int TICKS_LOW = 450;
    public static int TICKS_HIGH = 1200;
    public static int TICKS_MAX = 1250;

    public static double GRIP_OPEN = 0.65;
    public static double GRIP_CLOSED = 0.30;

    private final PositionalMotor<Level> lift;
    private final PositionalServo<Grip> claw;

    public ExampleLift(HardwareMap hardwareMap) {
        lift = new PositionalMotor<>(hardwareMap, "lift", Level.class)
                .preset(Level.DOWN, TICKS_DOWN)
                .preset(Level.LOW, TICKS_LOW)
                .preset(Level.HIGH, TICKS_HIGH)
                // Soft limits mean a mistyped preset cannot drive the lift through its hard stop.
                .limits(TICKS_DOWN, TICKS_MAX);

        claw = new PositionalServo<>(hardwareMap, "claw", Grip.class)
                .preset(Grip.OPEN, GRIP_OPEN)
                .preset(Grip.CLOSED, GRIP_CLOSED);
    }

    /** False if either device is missing from the configuration. */
    public boolean isAvailable() {
        return lift.isAvailable() && claw.isAvailable();
    }

    /** Call once per loop from {@code Robot.writeActuators()}. */
    public void update() {
        lift.update();
        claw.update();
    }

    public Level getLevel() {
        return lift.getState();
    }

    public Grip getGrip() {
        return claw.getState();
    }

    public boolean isAtLevel() {
        return lift.atState();
    }

    // ---- Intent-level commands ----

    /**
     * Grips, then raises. Sequenced in that order on purpose: raising an open claw drops the game
     * piece, and no caller should have to remember that.
     */
    public Command scoreAt(Level level) {
        return sequential(
                claw.goTo(Grip.CLOSED),
                lift.goTo(level)
        );
    }

    public Command scoreHigh() {
        return scoreAt(Level.HIGH);
    }

    /**
     * Releases, then lowers. The claw must finish opening before the lift moves, or the piece rides
     * back down with it — which is why the release is its own step rather than a parallel group.
     */
    public Command releaseAndRetract() {
        return sequential(
                claw.goTo(Grip.OPEN),
                lift.goTo(Level.DOWN)
        );
    }

    /** Direct access, for OpModes that genuinely need to drive one half on its own. */
    public PositionalMotor<Level> getLift() {
        return lift;
    }

    public PositionalServo<Grip> getClaw() {
        return claw;
    }
}
