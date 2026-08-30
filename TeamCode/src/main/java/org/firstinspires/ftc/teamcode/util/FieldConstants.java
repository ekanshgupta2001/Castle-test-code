package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Field geometry and alliance mirroring, in Pedro coordinates.
 *
 * <h2>Coordinate system</h2>
 * Origin at a field corner, 144" square, X right, Y up, headings in radians CCW from +X.
 *
 * <h2>How to use this file</h2>
 * Every pose below is written for <b>BLUE</b>. Ask for the red version with
 * {@link #forAlliance(Pose, Alliance)} rather than writing a second copy — one source of truth per
 * location means a measurement correction is a one-line change instead of a hunt for the mirrored
 * twin someone forgot to update.
 *
 * <pre>
 *   Pose start = FieldConstants.forAlliance(FieldConstants.startPose(pos), alliance);
 * </pre>
 *
 * <h2>These numbers are placeholders</h2>
 * BIOBUZZ field geometry is not encoded here yet. Everything marked TODO is a guess that exists so
 * the autonomous structure compiles and can be dry-run; <b>measure the real field and replace them
 * before trusting any path.</b> The mirroring math is real and tested — only the coordinates are not.
 */
@Configurable
public final class FieldConstants {
    private FieldConstants() {}

    /** Standard FTC field: 144 inches on a side. */
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_CENTER_INCHES = FIELD_SIZE_INCHES / 2;

    /**
     * Which way {@link #forAlliance} mirrors.
     *
     * <p>Set to match how the real field is laid out. Most FTC fields are symmetric across one axis;
     * pick the one that maps a blue location onto the corresponding red one. Getting this wrong
     * produces an autonomous that works perfectly on one alliance and drives into a wall on the other.
     */
    public static boolean MIRROR_ACROSS_X = true;

    // ---- Placeholder poses (TODO: measure on the real field) ----

    /** TODO placeholder. Blue-side starting pose for the left start position. */
    public static Pose BLUE_START_LEFT = new Pose(12, 60, Math.toRadians(0));
    /** TODO placeholder. Blue-side starting pose for the right start position. */
    public static Pose BLUE_START_RIGHT = new Pose(12, 84, Math.toRadians(0));
    /** TODO placeholder. Somewhere useful to drive to during auto. */
    public static Pose BLUE_STAGING = new Pose(36, 72, Math.toRadians(0));
    /** TODO placeholder. Where a game piece gets scored. */
    public static Pose BLUE_SCORE = new Pose(24, 120, Math.toRadians(90));
    /** TODO placeholder. Ending position that does not block an alliance partner. */
    public static Pose BLUE_PARK = new Pose(60, 12, Math.toRadians(270));

    /** Blue-side start pose for a given start position. */
    public static Pose startPose(StartPosition position) {
        return position == StartPosition.LEFT ? BLUE_START_LEFT : BLUE_START_RIGHT;
    }

    // ---- Mirroring ----

    /**
     * Converts a blue-side pose to the given alliance.
     *
     * <p>Blue is the identity, so blue poses can be written directly as measured.
     */
    public static Pose forAlliance(Pose bluePose, Alliance alliance) {
        if (bluePose == null) return null;
        if (alliance == Alliance.BLUE) return bluePose;
        return MIRROR_ACROSS_X ? mirrorAcrossX(bluePose) : mirrorAcrossY(bluePose);
    }

    /**
     * Reflects across the vertical centre line {@code x = 72}.
     *
     * <p>A direction {@code (cos h, sin h)} becomes {@code (-cos h, sin h)}, which is heading
     * {@code pi - h}. Mirroring the position without mirroring the heading is the classic way to end
     * up facing backwards on one alliance only.
     */
    public static Pose mirrorAcrossX(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                FIELD_SIZE_INCHES - pose.getX(),
                pose.getY(),
                VisionMath.normalizeAngle(Math.PI - pose.getHeading()));
    }

    /** Reflects across the horizontal centre line {@code y = 72}; heading becomes {@code -h}. */
    public static Pose mirrorAcrossY(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                pose.getX(),
                FIELD_SIZE_INCHES - pose.getY(),
                VisionMath.normalizeAngle(-pose.getHeading()));
    }

    /** True when a coordinate pair lies on the field. Used to reject impossible vision fixes. */
    public static boolean isInsideField(double x, double y) {
        return x >= 0 && x <= FIELD_SIZE_INCHES && y >= 0 && y <= FIELD_SIZE_INCHES;
    }

    /** True when a pose lies on the field. */
    public static boolean isInsideField(Pose pose) {
        return pose != null && isInsideField(pose.getX(), pose.getY());
    }
}
