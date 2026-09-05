package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Every gamepad binding, and the help card that describes them, from one definition.
 *
 * <h2>Why this is an enum and not a pile of if-statements</h2>
 * The bindings were previously written twice: once as {@code gamepad1.aWasPressed()} in the input
 * handler, and once as a hand-typed string in the init-phase help text. Nothing connected them, so
 * re-binding a button silently left the card lying to the drivers — and the card is the only thing
 * a new driver has at 9am on competition day.
 *
 * <p>Here the button, the label, and the description are one object. {@link #helpLines()} renders
 * the card from the same constants {@code Teleop} reads, so the two cannot disagree.
 *
 * <h2>Adding a control</h2>
 * Add an entry. It appears on the help card automatically; wire it in {@code Teleop} with
 * {@code CONTROL.wasPressed(gamepad1, gamepad2)}. Order here is the order shown on the card, so
 * group related actions together.
 *
 * <p>Analog inputs (sticks, triggers) are listed with a {@code null} test — they are documented
 * here but read directly, because there is no edge to detect.
 */
public enum Controls {
    // ---- Driver (gamepad 1): anything that moves the robot ----
    DRIVE(Pad.DRIVER, "sticks", "drive", null),
    SLOW_MODE(Pad.DRIVER, "L-trigger", "precision slow mode", null),
    TOGGLE_DRIVE_FRAME(Pad.DRIVER, "options", "field / robot centric", Gamepad::optionsWasPressed),
    RESET_HEADING(Pad.DRIVER, "Y", "re-zero field heading", Gamepad::yWasPressed),
    ABORT(Pad.DRIVER, "BACK", "abort macro (or just move a stick)", Gamepad::backWasPressed),

    COLLECT(Pad.DRIVER, "A", "collect pollen", Gamepad::aWasPressed),
    ALIGN_SERVO(Pad.DRIVER, "X", "servo-align to pollen", Gamepad::xWasPressed),
    ALIGN_PATH(Pad.DRIVER, "B", "path-align to pollen", Gamepad::bWasPressed),
    RELOCALIZE(Pad.DRIVER, "RB", "relocalize from AprilTag", Gamepad::rightBumperWasPressed),

    SNAP_90(Pad.DRIVER, "dpad up", "snap to 90 deg", Gamepad::dpadUpWasPressed),
    SNAP_0(Pad.DRIVER, "dpad right", "snap to 0 deg", Gamepad::dpadRightWasPressed),
    SNAP_270(Pad.DRIVER, "dpad down", "snap to 270 deg", Gamepad::dpadDownWasPressed),
    SNAP_180(Pad.DRIVER, "dpad left", "snap to 180 deg", Gamepad::dpadLeftWasPressed),

    // ---- Operator (gamepad 2): mechanisms and diagnostics ----
    INTAKE(Pad.OPERATOR, "RB", "run intake", Gamepad::rightBumperWasPressed),
    OUTTAKE(Pad.OPERATOR, "LB", "run intake backwards", Gamepad::leftBumperWasPressed),
    EJECT(Pad.OPERATOR, "B", "eject at full speed", Gamepad::bWasPressed),
    STOP_INTAKE(Pad.OPERATOR, "X", "stop intake", Gamepad::xWasPressed),
    CAPTURE_AND_HOLD(Pad.OPERATOR, "Y", "intake until captured, then hold", Gamepad::yWasPressed),

    LIFT_HIGH(Pad.OPERATOR, "dpad up", "lift to HIGH", Gamepad::dpadUpWasPressed),
    LIFT_LOW(Pad.OPERATOR, "dpad right", "lift to LOW", Gamepad::dpadRightWasPressed),
    LIFT_DOWN(Pad.OPERATOR, "dpad down", "lift to DOWN", Gamepad::dpadDownWasPressed),
    TOGGLE_GRIP(Pad.OPERATOR, "dpad left", "open / close claw", Gamepad::dpadLeftWasPressed),

    TOGGLE_DEBUG(Pad.OPERATOR, "BACK", "toggle debug telemetry", Gamepad::backWasPressed);

    /** Which driver holds this control. */
    public enum Pad { DRIVER, OPERATOR }

    private final Pad pad;
    private final String button;
    private final String description;
    private final Predicate<Gamepad> edge;

    Controls(Pad pad, String button, String description, Predicate<Gamepad> edge) {
        this.pad = pad;
        this.button = button;
        this.description = description;
        this.edge = edge;
    }

    /**
     * Whether this control was pressed since the last check.
     *
     * <p>Takes both gamepads and picks the right one, so callers never have to remember which pad
     * a control lives on — moving a binding between pads is then a one-line change here.
     *
     * <p>Always false for analog controls, which are read directly.
     */
    public boolean wasPressed(Gamepad driver, Gamepad operator) {
        if (edge == null) return false;
        return edge.test(pad == Pad.DRIVER ? driver : operator);
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
