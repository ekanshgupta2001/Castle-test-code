package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class AutoSelector {
    private Alliance alliance = Alliance.RED;
    private StartPosition start = StartPosition.LEFT;
    private boolean confirmed = false;

    public void poll(Gamepad gamepad) {
        if (confirmed) return;

        if (gamepad.dpadLeftWasPressed() || gamepad.dpadRightWasPressed()) {
            alliance = alliance.opposite();
        }
        if (gamepad.dpadUpWasPressed() || gamepad.dpadDownWasPressed()) {
            start = cycle(start);
        }
        if (gamepad.aWasPressed()) {
            confirmed = true;
        }
    }

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
        sb.append(confirmed ? "[CONFIRMED]" : "[choose]").append('\n');
        sb.append("Alliance: ").append(alliance).append("   (dpad ←/→)\n");
        sb.append("Start:    ").append(start).append("   (dpad ↑/↓)\n");
        sb.append(confirmed ? "" : "Press A to confirm.");
        return sb.toString();
    }

    private static StartPosition cycle(StartPosition s) {
        StartPosition[] vals = StartPosition.values();
        return vals[(s.ordinal() + 1) % vals.length];
    }
}
