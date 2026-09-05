package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

/**
 * Init-phase menu for choosing alliance and starting position.
 *
 * <p>Dpad left/right flips the alliance, dpad up/down cycles the start position, A confirms, and
 * B un-confirms so a wrong pick can be corrected without restarting the OpMode.
 *
 * <p>Relies on the SDK's {@code *WasPressed()} one-shots, which latch on the rising edge and clear
 * when read. That makes them rate-independent — but it also means they must be read <em>every</em>
 * loop. See {@link #poll}.
 */
public class AutoSelector {
    private Alliance alliance = Alliance.RED;
    private StartPosition start = StartPosition.LEFT;
    private boolean confirmed = false;

    public void poll(Gamepad gamepad) {
        // Read every edge unconditionally, even when confirmed. Returning early instead would leave
        // presses latched in the gamepad; they would all fire at once after the next B press,
        // flipping the alliance and re-confirming before the user could touch anything.
        boolean left = gamepad.dpadLeftWasPressed();
        boolean right = gamepad.dpadRightWasPressed();
        boolean up = gamepad.dpadUpWasPressed();
        boolean down = gamepad.dpadDownWasPressed();
        boolean confirm = gamepad.aWasPressed();
        boolean change = gamepad.bWasPressed();

        if (confirmed) {
            if (change) unlock();
            return;
        }

        if (left || right) {
            alliance = alliance.opposite();
        }
        if (up) {
            start = cycle(start, 1);
        } else if (down) {
            start = cycle(start, -1);
        }
        if (confirm) {
            confirmed = true;
        }
    }

    /** Drops the confirmation so the selection can be changed. Bound to B in {@link #poll}. */
    public void unlock() {
        confirmed = false;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public StartPosition getStart() {
        return start;
    }

    public boolean isConfirmed() {
        return confirmed;
    }

    public String render() {
        StringBuilder sb = new StringBuilder();
        sb.append(confirmed ? "[CONFIRMED]   (B to change)" : "[choose]").append('\n');
        sb.append("Alliance: ").append(alliance).append("   (dpad left/right)\n");
        sb.append("Start:    ").append(start).append("   (dpad up/down)\n");
        sb.append(confirmed ? "" : "Press A to confirm.");
        return sb.toString();
    }

    /** Steps through the enum in either direction, wrapping at both ends. */
    private static StartPosition cycle(StartPosition s, int step) {
        StartPosition[] vals = StartPosition.values();
        int next = (s.ordinal() + step % vals.length + vals.length) % vals.length;
        return vals[next];
    }
}
