package org.firstinspires.ftc.teamcode.util.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

/**
 * Tests for alliance mirroring.
 *
 * <p>The coordinates in {@link FieldConstants} are placeholders, but the mirroring math is real and
 * is the part that silently breaks one alliance while the other looks fine — so it gets tested.
 */
public class FieldConstantsTest {
    private static final double EPS = 1e-9;

    @Test
    public void blueIsTheIdentity() {
        Pose p = new Pose(12, 60, Math.toRadians(30));
        assertSame(p, FieldConstants.forAlliance(p, Alliance.BLUE));
    }

    @Test
    public void mirrorAcrossXReflectsPositionAndHeading() {
        Pose p = new Pose(12, 60, 0);
        Pose m = FieldConstants.mirrorAcrossX(p);
        assertEquals(132, m.getX(), EPS);          // 144 - 12
        assertEquals(60, m.getY(), EPS);           // unchanged
        assertEquals(Math.PI, m.getHeading(), 1e-9); // facing +X becomes facing -X
    }

    @Test
    public void mirrorAcrossXIsItsOwnInverse() {
        Pose p = new Pose(30, 100, Math.toRadians(37));
        Pose back = FieldConstants.mirrorAcrossX(FieldConstants.mirrorAcrossX(p));
        assertEquals(p.getX(), back.getX(), 1e-9);
        assertEquals(p.getY(), back.getY(), 1e-9);
        assertEquals(p.getHeading(), back.getHeading(), 1e-9);
    }

    @Test
    public void mirrorAcrossYReflectsPositionAndHeading() {
        Pose p = new Pose(12, 60, Math.toRadians(90));
        Pose m = FieldConstants.mirrorAcrossY(p);
        assertEquals(12, m.getX(), EPS);
        assertEquals(84, m.getY(), EPS);           // 144 - 60
        assertEquals(Math.toRadians(270), m.getHeading(), 1e-9);  // +Y becomes -Y
    }

    @Test
    public void mirrorAcrossYIsItsOwnInverse() {
        Pose p = new Pose(30, 100, Math.toRadians(200));
        Pose back = FieldConstants.mirrorAcrossY(FieldConstants.mirrorAcrossY(p));
        assertEquals(p.getX(), back.getX(), 1e-9);
        assertEquals(p.getY(), back.getY(), 1e-9);
        assertEquals(p.getHeading(), back.getHeading(), 1e-9);
    }

    @Test
    public void mirroredHeadingIsAlwaysNormalized() {
        for (double deg = 0; deg < 360; deg += 15) {
            Pose p = new Pose(20, 20, Math.toRadians(deg));
            double hx = FieldConstants.mirrorAcrossX(p).getHeading();
            double hy = FieldConstants.mirrorAcrossY(p).getHeading();
            assertTrue("mirrorX out of range at " + deg, hx >= 0 && hx < 2 * Math.PI);
            assertTrue("mirrorY out of range at " + deg, hy >= 0 && hy < 2 * Math.PI);
        }
    }

    @Test
    public void mirroringPreservesDistanceFromTheCentreLine() {
        Pose p = new Pose(20, 60, 0);
        Pose m = FieldConstants.mirrorAcrossX(p);
        double before = Math.abs(p.getX() - FieldConstants.FIELD_CENTER_INCHES);
        double after = Math.abs(m.getX() - FieldConstants.FIELD_CENTER_INCHES);
        assertEquals(before, after, EPS);
    }

    @Test
    public void aPoseOnTheCentreLineIsUnmoved() {
        Pose p = new Pose(FieldConstants.FIELD_CENTER_INCHES, 40, Math.toRadians(90));
        Pose m = FieldConstants.mirrorAcrossX(p);
        assertEquals(p.getX(), m.getX(), EPS);
        // Facing straight up is unchanged by a left/right mirror.
        assertEquals(Math.toRadians(90), m.getHeading(), 1e-9);
    }

    @Test
    public void fieldBoundsCheck() {
        assertTrue(FieldConstants.isInsideField(0, 0));
        assertTrue(FieldConstants.isInsideField(144, 144));
        assertTrue(FieldConstants.isInsideField(72, 72));
        assertFalse(FieldConstants.isInsideField(-1, 72));
        assertFalse(FieldConstants.isInsideField(72, 145));
    }

    @Test
    public void nullIsPassedThroughRatherThanThrowing() {
        assertEquals(null, FieldConstants.forAlliance(null, Alliance.RED));
        assertEquals(null, FieldConstants.mirrorAcrossX(null));
        assertEquals(null, FieldConstants.mirrorAcrossY(null));
        assertFalse(FieldConstants.isInsideField(null));
    }
}
