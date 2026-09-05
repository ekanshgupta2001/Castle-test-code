package org.firstinspires.ftc.teamcode.game;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;

/**
 * Everything the code knows about this season's game piece.
 *
 * <p>The subsystems are written in terms of a generic "piece" and "blob": the intake reports
 * {@code hasPiece()}, the camera reports {@code hasStableBlob()}. What a pollen looks like to the
 * colour sensor, how tall it is for the camera's ground-plane geometry, and what to call it on the
 * driver's screen all live here, so next season's game is one file to replace rather than a
 * search across the tree.
 *
 * <p>{@code Robot} is the only production class that reads this; it wires the values into the
 * subsystems each loop, the same way {@code Intake} pushes its thresholds into the jam detector,
 * so live edits on the dashboard take effect immediately.
 */
@Configurable
public final class Pollen {
    private Pollen() {}

    /** What the drivers call it. Used in help text and telemetry. */
    public static final String NAME = "pollen";

    /** Hue of a pollen in degrees. Yellow sits near 55. Tune on the real field. */
    public static float HUE_DEGREES = 55f;
    public static float HUE_TOLERANCE = 25f;
    public static float MIN_SATURATION = 0.45f;
    public static float MIN_VALUE = 0.20f;

    /** Height of the piece's centre above the floor, for the camera's ray-to-floor intersection. */
    public static double HEIGHT_INCHES = 1.5;

    /**
     * True when the colour sensor is looking at something pollen-coloured.
     *
     * <p>Thresholds hue first, gated by saturation and brightness. A brightness-only test would
     * fire on any bright object — a white field wall reads as "captured".
     */
    public static boolean isAtSensor(ColorSensor sensor) {
        return sensor.matchesHue(HUE_DEGREES, HUE_TOLERANCE, MIN_SATURATION, MIN_VALUE);
    }
}
