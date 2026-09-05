package org.firstinspires.ftc.teamcode.subsystems.templates;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

/**
 * Worked example: a two-stage scoring mechanism built entirely from the templates.
 *
 * <p><b>This is wired into {@link org.firstinspires.ftc.teamcode.Robot} like any other subsystem.</b>
 * There is no lift on the robot yet, so {@code Hardware.get} returns null, {@link #isAvailable()} is
 * false, and every call below no-ops — but the wiring is real: {@code Robot} constructs it,
 * {@code writeActuators()} ticks it, and {@code Teleop} binds the operator dpad to it. Bolt on a
 * lift, add its two names to {@code HardwareNames}, and it works with no code change.
 *
 * <p>That is deliberate. A pattern nothing uses is a pattern nobody trusts — you would have had to
 * guess whether it actually fits the rest of the codebase. Read {@code Robot}, {@code Teleop} and
 * this file together and you can see the whole path from a button press to a mechanism.
 *
 * <p>It is still the file to copy when you build a <em>second</em> mechanism.
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
 * <h2>How it is wired</h2>
 * <pre>
 *   Robot()                  lift = new ExampleLift(hardwareMap);
 *   Robot.writeActuators()   lift.update();
 *   Teleop.handleOperatorInput()
 *                            if (Controls.LIFT_HIGH.wasPressed(gamepad1, gamepad2))
 *                                robot.lift.goToLevel(Level.HIGH).schedule();
 * </pre>
 *
 * <p>Three lines, the same three any subsystem needs. The {@code isAvailable()} guard in
 * {@code Teleop} is what makes it safe to commit before the hardware exists.
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
        lift = new PositionalMotor<>(hardwareMap, HardwareNames.LIFT_MOTOR, Level.class)
                .preset(Level.DOWN, TICKS_DOWN)
                .preset(Level.LOW, TICKS_LOW)
                .preset(Level.HIGH, TICKS_HIGH)
                // Soft limits mean a mistyped preset cannot drive the lift through its hard stop.
                .limits(TICKS_DOWN, TICKS_MAX);

        claw = new PositionalServo<>(hardwareMap, HardwareNames.CLAW_SERVO, Grip.class)
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

    /** Moves the lift alone, leaving the claw where it is. */
    public Command goToLevel(Level level) {
        return lift.goTo(level);
    }

    /** Opens a closed claw and closes an open one. */
    public Command toggleGrip() {
        return claw.goTo(getGrip() == Grip.CLOSED ? Grip.OPEN : Grip.CLOSED);
    }

    /** Direct access, for OpModes that genuinely need to drive one half on its own. */
    public PositionalMotor<Level> getLift() {
        return lift;
    }

    public PositionalServo<Grip> getClaw() {
        return claw;
    }
}
