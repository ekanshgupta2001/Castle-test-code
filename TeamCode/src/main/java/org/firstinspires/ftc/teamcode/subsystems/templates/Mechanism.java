package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.pedropathing.ivy.Command;

/**
 * A mechanism that moves between a fixed set of named positions.
 *
 * <h2>Why the state is an enum</h2>
 *
 * Almost every FTC mechanism is really a small state machine: a lift is DOWN, LOW, or HIGH; a claw
 * is OPEN or CLOSED. Naming those states in an {@code enum} rather than with strings means a typo is
 * a <b>compile error</b> instead of a robot that silently does nothing in the middle of a match. It
 * also makes the set of legal positions discoverable — {@code LiftState.values()} lists them, and
 * your IDE autocompletes them.
 *
 * <pre>
 *   public enum LiftState { DOWN, LOW, HIGH }
 *
 *   lift.goTo(LiftState.HIGH).schedule();   // typo here will not compile
 * </pre>
 *
 * <h2>Implementing this</h2>
 *
 * {@link PositionalMotor} and {@link PositionalServo} already implement it for the two common cases
 * (an encoder motor and a servo), so most mechanisms need no new code at all — just an enum and some
 * preset values. See {@code ExampleLift} for a worked example.
 *
 * @param <S> the enum naming this mechanism's positions
 */
public interface Mechanism<S extends Enum<S>> {

    /**
     * Builds a command that moves to {@code state}.
     *
     * <p>Returns the command; it does <b>not</b> run it. Call {@code .schedule()}, or compose it
     * into a larger sequence. Nothing happens until the scheduler runs it.
     */
    Command goTo(S state);

    /**
     * The state most recently commanded — not necessarily the one the mechanism has reached.
     *
     * <p>Use {@link #atState()} to ask whether it actually got there. The two differ for the whole
     * duration of every move, which is exactly when it matters.
     */
    S getState();

    /** True once the mechanism has physically arrived at {@link #getState()}. */
    boolean atState();

    /**
     * Per-loop hardware tick. Call from {@code Robot.writeActuators()}.
     *
     * <p>May legitimately do nothing — a servo has no feedback loop to run, and a motor in
     * RUN_TO_POSITION is closed-loop inside the hub. Implementations that do nothing should say so.
     */
    void update();
}
