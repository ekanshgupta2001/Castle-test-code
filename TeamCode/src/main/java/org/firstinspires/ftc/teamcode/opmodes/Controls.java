package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.game.Pollen;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/**
 * Every gamepad binding, and the help card that describes them, from one definition.
 *
 * <h2>Why this is an enum and not a pile of if-statements</h2>
 * A binding written as {@code gamepad1.aWasPressed()} in the handler and again as a hand-typed
 * string on the help card has nothing connecting the two, and the card is the only thing a new
 * driver has at 9am on competition day. Here the button, the label, and the description are one
 * object. {@link #helpLines()} renders
 * the card from the same constants {@code Teleop} reads, so the two cannot disagree.
 *
 * <h2>Adding a control</h2>
 * Add an entry. It appears on the help card automatically; wire it in {@code Teleop} with
 * {@code CONTROL.wasPressed(gamepad1, gamepad2)} for a button or
 * {@code CONTROL.axis(gamepad1, gamepad2)} for a stick or trigger. Order here is the order shown on
 * the card, so group related actions together.
 *
 * <p>Analog controls carry an axis function instead of an edge test. Sign conventions live in that
 * function — the SDK reports stick-up as negative, so {@link #DRIVE_FORWARD} negates it and every
 * caller gets "forward is positive" without remembering why.
 */
public enum Controls {
    // ---- Driver (gamepad 1): anything that moves the robot ----
    DRIVE_FORWARD(Pad.DRIVER, "L-stick up", "drive forward", null, gp -> -gp.left_stick_y),
    DRIVE_STRAFE(Pad.DRIVER, "L-stick left", "strafe left", null, gp -> -gp.left_stick_x),
    DRIVE_TURN(Pad.DRIVER, "R-stick left", "turn left", null, gp -> -gp.right_stick_x),
    SLOW_MODE(Pad.DRIVER, "L-trigger", "precision slow mode", null, gp -> gp.left_trigger),
    TOGGLE_DRIVE_FRAME(Pad.DRIVER, "options", "field / robot centric", Gamepad::optionsWasPressed, null),
    RESET_HEADING(Pad.DRIVER, "Y", "re-zero field heading", Gamepad::yWasPressed, null),
    ABORT(Pad.DRIVER, "BACK", "abort macro (or just move a stick)", Gamepad::backWasPressed, null),

    COLLECT(Pad.DRIVER, "A", "collect " + Pollen.NAME, Gamepad::aWasPressed, null),
    ALIGN_SERVO(Pad.DRIVER, "X", "servo-align to " + Pollen.NAME, Gamepad::xWasPressed, null),
    ALIGN_PATH(Pad.DRIVER, "B", "path-align to " + Pollen.NAME, Gamepad::bWasPressed, null),
    RELOCALIZE(Pad.DRIVER, "RB", "relocalize from AprilTag", Gamepad::rightBumperWasPressed, null),

    SNAP_90(Pad.DRIVER, "dpad up", "snap to 90 deg", Gamepad::dpadUpWasPressed, null),
    SNAP_0(Pad.DRIVER, "dpad right", "snap to 0 deg", Gamepad::dpadRightWasPressed, null),
    SNAP_270(Pad.DRIVER, "dpad down", "snap to 270 deg", Gamepad::dpadDownWasPressed, null),
    SNAP_180(Pad.DRIVER, "dpad left", "snap to 180 deg", Gamepad::dpadLeftWasPressed, null),

    // ---- Operator (gamepad 2): mechanisms and diagnostics ----
    INTAKE(Pad.OPERATOR, "RB", "run intake", Gamepad::rightBumperWasPressed, null),
    OUTTAKE(Pad.OPERATOR, "LB", "run intake backwards", Gamepad::leftBumperWasPressed, null),
    EJECT(Pad.OPERATOR, "B", "eject at full speed", Gamepad::bWasPressed, null),
    STOP_INTAKE(Pad.OPERATOR, "X", "stop intake", Gamepad::xWasPressed, null),
    CAPTURE_AND_HOLD(Pad.OPERATOR, "Y", "intake until captured, then hold", Gamepad::yWasPressed, null),

    LIFT_HIGH(Pad.OPERATOR, "dpad up", "lift to HIGH", Gamepad::dpadUpWasPressed, null),
    LIFT_LOW(Pad.OPERATOR, "dpad right", "lift to LOW", Gamepad::dpadRightWasPressed, null),
    LIFT_DOWN(Pad.OPERATOR, "dpad down", "lift to DOWN", Gamepad::dpadDownWasPressed, null),
    TOGGLE_GRIP(Pad.OPERATOR, "dpad left", "open / close claw", Gamepad::dpadLeftWasPressed, null),

    TOGGLE_DEBUG(Pad.OPERATOR, "BACK", "toggle debug telemetry", Gamepad::backWasPressed, null);

    /** Which driver holds this control. */
    public enum Pad { DRIVER, OPERATOR }

    private final Pad pad;
    private final String button;
    private final String description;
    private final Predicate<Gamepad> edge;
    private final ToDoubleFunction<Gamepad> axis;

    /**
     * A button carries an {@code edge} test and a null {@code axis}; a stick or trigger the
     * reverse. One constructor rather than two overloads because an implicitly typed lambda is
     * ambiguous between {@code Predicate} and {@code ToDoubleFunction} in Java 8.
     */
    Controls(Pad pad, String button, String description,
             Predicate<Gamepad> edge, ToDoubleFunction<Gamepad> axis) {
        this.pad = pad;
        this.button = button;
        this.description = description;
        this.edge = edge;
        this.axis = axis;
    }

    /**
     * Whether this control was pressed since the last check.
     *
     * <p>Takes both gamepads and picks the right one, so callers never have to remember which pad
     * a control lives on — moving a binding between pads is then a one-line change here.
     *
     * <p>Always false for analog controls; read those with {@link #axis}.
     */
    public boolean wasPressed(Gamepad driver, Gamepad operator) {
        if (edge == null) return false;
        return edge.test(pad == Pad.DRIVER ? driver : operator);
    }

    /**
     * The current value of a stick or trigger, with the sign convention already applied.
     *
     * <p>Always 0 for button controls; read those with {@link #wasPressed}.
     */
    public double axis(Gamepad driver, Gamepad operator) {
        if (axis == null) return 0;
        return axis.applyAsDouble(pad == Pad.DRIVER ? driver : operator);
    }

    public boolean isAnalog() {
        return axis != null;
    }

    public Pad pad() {
        return pad;
    }

    public String button() {
        return button;
    }

    public String description() {
        return description;
    }

    /**
     * The init-phase help card, generated from the bindings above.
     *
     * <p>Several controls share a line where they form one group (the four snap headings, the four
     * lift presets), because a driver reads "dpad = snap to heading" faster than four separate lines.
     */
    public static List<String> helpLines() {
        List<String> lines = new ArrayList<>();
        for (Pad pad : Pad.values()) {
            lines.add(pad == Pad.DRIVER ? "DRIVER (gamepad 1)" : "OPERATOR (gamepad 2)");
            StringBuilder row = new StringBuilder("  ");
            for (Controls c : values()) {
                if (c.pad != pad) continue;
                String entry = c.button + "=" + c.description;
                if (row.length() + entry.length() > 70) {
                    lines.add(row.toString());
                    row = new StringBuilder("  ");
                }
                if (row.length() > 2) row.append("   ");
                row.append(entry);
            }
            if (row.length() > 2) lines.add(row.toString());
        }
        return lines;
    }
}
