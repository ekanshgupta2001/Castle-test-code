package org.firstinspires.ftc.teamcode.game;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

/**
 * Where things are on this season's field, in Pedro coordinates.
 *
 * <p>Every pose is written for <b>BLUE</b>. Ask for the red version with
 * {@link FieldConstants#forAlliance} rather than writing a second copy — one source of truth per
 * location means a measurement correction is a one-line change instead of a hunt for the mirrored
 * twin someone forgot to update. The field's size and the mirroring math are game-independent and
 * live in {@link FieldConstants}; only the locations change between seasons, and they are all here.
 *
 * <h2>These numbers are placeholders</h2>
 * BIOBUZZ field geometry is not encoded here yet. Everything marked TODO is a guess that exists so
 * the autonomous structure compiles and can be dry-run; <b>measure the real field and replace them
 * before trusting any path.</b>
 */
@Configurable
public final class FieldPoses {
    private FieldPoses() {}

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
}
