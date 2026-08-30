package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

/**
 * Renders the robot, its paths, and its pose history onto the Panels dashboard field view.
 *
 * <p>Originally a package-private class inside {@code pedroPathing.Tuning}, which meant only the
 * tuning OpModes could draw. It lives here so teleop, autonomous, and vision debugging can all
 * visualise what the robot thinks is happening — usually the fastest way to see that a pose
 * estimate has gone wrong.
 *
 * <p>Call {@link #init()} once, draw during the loop, then {@link #sendPacket()} to push the frame.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style historyLook = new Style("", "#4CAF50", 0.75);
    private static final Style targetLook = new Style("", "#FF9800", 0.75);

    /** Prepares the Panels field to use Pedro's coordinate offsets. */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /** Draws the follower's current path, its closest point, the pose history, and the robot. */
    public static void drawDebug(Follower follower) {
        if (follower == null) return;
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(),
                    follower.getCurrentPath().getHeadingGoal(
                            follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /** Draws a robot at a Pose, with heading shown as a line. */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /** Draws a vision target or path goal in a contrasting colour. */
    public static void drawTarget(Pose pose) {
        if (pose == null) return;
        panelsField.setStyle(targetLook);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(3);
    }

    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        if (poseTracker == null) return;
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i],
                    poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1],
                    poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /** Pushes the accumulated drawing commands to the dashboard. */
    public static void sendPacket() {
        panelsField.update();
    }
}
