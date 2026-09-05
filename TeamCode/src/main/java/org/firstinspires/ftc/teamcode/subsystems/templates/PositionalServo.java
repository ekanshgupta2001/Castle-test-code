package org.firstinspires.ftc.teamcode.subsystems.templates;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.Hardware;

import java.util.EnumMap;
import java.util.Map;

/**
 * A servo driven to named positions.
 *
 * <pre>
 *   public enum ClawState { OPEN, CLOSED }
 *
 *   claw = new PositionalServo&lt;&gt;(hardwareMap, "claw", ClawState.class)
 *           .preset(ClawState.OPEN, 0.65)
 *           .preset(ClawState.CLOSED, 0.30);
 *
 *   claw.goTo(ClawState.CLOSED).schedule();
 * </pre>
 *
 * <h2>Modelling travel time</h2>
 * A servo has no position feedback, so "did it arrive?" cannot be measured — only waited out.
 * {@link #goTo} is therefore {@code sequential(instant(set), waitMs(travel))}, which holds the
 * subsystem's Ivy resource for the whole travel time. That is what stops the next command in a
 * sequence from acting on a mechanism that is still moving.
 *
 * <p>{@link #TRAVEL_MS_PER_UNIT} scales the wait with how far the servo actually has to go, so a
 * small nudge does not cost the same as a full sweep. A fixed delay wastes match time on short
 * moves and finishes early on long ones — which is worse, because the sequence then continues
 * against a servo still in motion.
 *
 * @param <S> the enum naming this mechanism's positions
 */
@Configurable
public class PositionalServo<S extends Enum<S>> implements Mechanism<S> {
    /** Milliseconds per unit of servo travel (full sweep is 1.0). Measure this on the real servo. */
    public static long TRAVEL_MS_PER_UNIT = 500;
    /** Floor on the travel wait, covering fixed overheads on very small moves. */
    public static long MIN_TRAVEL_MS = 60;

    private final Servo servo;
    private final Map<S, Double> presets;
    private double currentPos;
    private S state = null;

    public PositionalServo(HardwareMap hardwareMap, String name, Class<S> stateType) {
        this(hardwareMap, name, stateType, Double.NaN);
    }

    /**
     * @param initialPosition position to command at init, or {@link Double#NaN} to leave the servo
     *                        where it is. Prefer NaN unless you know the path is clear — a servo
     *                        commanded during init slams to that position before the match starts.
     */
    public PositionalServo(HardwareMap hardwareMap, String name, Class<S> stateType,
                           double initialPosition) {
        this.presets = new EnumMap<>(stateType);
        servo = Hardware.get(hardwareMap, Servo.class, name);
        if (servo == null) return;
        if (!Double.isNaN(initialPosition)) {
            setNow(initialPosition);
        } else {
            currentPos = servo.getPosition();
        }
    }

    /** False when the servo is missing from the robot configuration. All motion calls no-op. */
    public boolean isAvailable() {
        return servo != null;
    }

    /** Registers the position for one state. Chainable. */
    public PositionalServo<S> preset(S state, double position) {
        presets.put(state, Range.clip(position, 0.0, 1.0));
        return this;
    }

    /** Commands a raw position immediately, without waiting for travel. */
    public void setNow(double position) {
        if (servo == null) return;
        // Servo.setPosition clips internally; clip here too so the cached value matches reality
        // rather than reporting a position the servo never went to.
        currentPos = Range.clip(position, 0.0, 1.0);
        servo.setPosition(currentPos);
    }

    /** States without a preset are ignored rather than throwing — this runs inside a command. */
    public void setNow(S state) {
        Double pos = presets.get(state);
        if (pos == null) return;
        this.state = state;
        setNow(pos);
    }

    /** The last commanded position, after clipping. Servos cannot report where they actually are. */
    public double getCommandedPosition() {
        return currentPos;
    }

    @Override
    public S getState() {
        return state;
    }

    /**
     * Always true: a servo has no feedback, so arrival cannot be sensed, only assumed once the
     * travel wait in {@link #goTo} has elapsed.
     */
    @Override
    public boolean atState() {
        return true;
    }

    /** Intentionally empty: a servo is open-loop, with no per-loop work to do. */
    @Override
    public void update() {
    }

    /** Travel time for a move of the given distance, in servo units. */
    public long travelMsFor(double delta) {
        return Math.max(MIN_TRAVEL_MS, (long) (Math.abs(delta) * TRAVEL_MS_PER_UNIT));
    }

    @Override
    public Command goTo(S state) {
        Double target = presets.get(state);
        if (target == null) return Command.build().setDone(() -> true);
        return goToPosition(target, () -> setNow(state));
    }

    /** Same travel-time modelling as {@link #goTo}, for a raw position. */
    public Command goToCommand(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        return goToPosition(clipped, () -> setNow(clipped));
    }

    private Command goToPosition(double target, Runnable apply) {
        // Distance is measured when the command is BUILT, which is the best estimate available -
        // the servo may have moved by the time it runs, but only another command could have moved
        // it, and that command holds this same resource so it cannot overlap.
        long travel = travelMsFor(target - currentPos);
        return sequential(
                instant(apply).requiring(this),
                waitMs(travel)
        );
    }
}
