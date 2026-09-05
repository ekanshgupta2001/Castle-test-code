package org.firstinspires.ftc.teamcode.util.field;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.math.Angles;

/**
 * Field geometry and alliance mirroring, in Pedro coordinates. Game-independent.
 *
 * <h2>Coordinate system</h2>
 * Origin at a field corner, 144" square, X right, Y up, headings in radians CCW from +X.
 *
 * <h2>Where the season's locations live</h2>
 * Nothing here knows what is on the field. Start poses, scoring poses and parking spots are in
 * {@code game/FieldPoses}, written once for <b>BLUE</b> and mirrored on demand with
 * {@link #forAlliance(Pose, Alliance)}:
 *
 * <pre>
 *   Pose start = FieldConstants.forAlliance(FieldPoses.startPose(pos), alliance);
 * </pre>
 *
 * One source of truth per location means a measurement correction is a one-line change instead
 * of a hunt for the mirrored twin someone forgot to update. The mirroring math is unit-tested in
 * {@code FieldConstantsTest}.
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
                Angles.normalizeAngle(Math.PI - pose.getHeading()));
    }

    /** Reflects across the horizontal centre line {@code y = 72}; heading becomes {@code -h}. */
    public static Pose mirrorAcrossY(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                pose.getX(),
                FIELD_SIZE_INCHES - pose.getY(),
                Angles.normalizeAngle(-pose.getHeading()));
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
