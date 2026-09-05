package org.firstinspires.ftc.teamcode.game;

import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;
import org.junit.Test;

/** Sanity checks on the season's poses. They are placeholders, but they must at least be legal. */
public class FieldPosesTest {
    @Test
    public void everyPoseAndItsMirrorStaysOnTheField() {
        Pose[] samples = {
                FieldPoses.BLUE_START_LEFT,
                FieldPoses.BLUE_START_RIGHT,
                FieldPoses.BLUE_STAGING,
                FieldPoses.BLUE_SCORE,
                FieldPoses.BLUE_PARK,
        };
        for (Pose p : samples) {
            assertTrue("blue pose off field: " + p, FieldConstants.isInsideField(p));
            assertTrue("red mirror off field: " + p,
                    FieldConstants.isInsideField(FieldConstants.forAlliance(p, Alliance.RED)));
        }
    }

    @Test
    public void startPosesAreDistinct() {
        Pose left = FieldPoses.startPose(StartPosition.LEFT);
        Pose right = FieldPoses.startPose(StartPosition.RIGHT);
        assertTrue("start positions must be distinct",
                left.getX() != right.getX() || left.getY() != right.getY());
    }
}
