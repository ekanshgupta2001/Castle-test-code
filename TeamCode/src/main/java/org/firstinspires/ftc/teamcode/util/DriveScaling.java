package org.firstinspires.ftc.teamcode.util;

public final class DriveScaling {
    public static double DEFAULT_DEADBAND = 0.07;
    public static double DEFAULT_EXPO = 2.0;
    public static double SLOW_MIN_SCALE = 0.3;
    public static double SLOW_THRESHOLD = 0.1;

    private DriveScaling() {}

    public static double applyDeadband(double raw, double deadband) {
        if (Math.abs(raw) < deadband) return 0;
        double sign = raw < 0 ? -1 : 1;
        return sign * (Math.abs(raw) - deadband) / (1 - deadband);
    }

    public static double applyDeadband(double raw) {
        return applyDeadband(raw, DEFAULT_DEADBAND);
    }

    public static double applyExpo(double raw, double exponent) {
        double sign = raw < 0 ? -1 : 1;
        return sign * Math.pow(Math.abs(raw), exponent);
    }

    public static double applyExpo(double raw) {
        return applyExpo(raw, DEFAULT_EXPO);
    }

    public static double shape(double raw) {
        return applyExpo(applyDeadband(raw));
    }

    public static double slowScale(double trigger, double minScale, double threshold) {
        if (trigger < threshold) return 1.0;
        double t = (trigger - threshold) / (1.0 - threshold);
        return 1.0 - t * (1.0 - minScale);
    }

    public static double slowScale(double trigger) {
        return slowScale(trigger, SLOW_MIN_SCALE, SLOW_THRESHOLD);
    }
}
