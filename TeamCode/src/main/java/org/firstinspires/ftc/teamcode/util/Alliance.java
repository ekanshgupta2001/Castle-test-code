package org.firstinspires.ftc.teamcode.util;

public enum Alliance {
    RED,
    BLUE;

    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }
}
