package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoSelector;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.firstinspires.ftc.teamcode.util.diagnostics.Drawing;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

/**
 * Autonomous OpMode: choose the alliance and start position during init, then run
 * {@link AutoRoutine}.
 *
 * <p>Init builds nothing but the selector. The routine itself is assembled in {@code start()},
 * after the alliance is known, as one Ivy command tree. The loop then does nothing but tick
 * sensors, the scheduler, and the follower — all the sequencing lives in {@link AutoRoutine},
 * which is why this class is short and that one is testable.
 *
 * <p>{@link PoseStorage} is written every loop, not once at the end, so teleop still inherits a
 * good pose if this OpMode is stopped early.
 */
@Configurable
@Autonomous(name = "Auto", group = "Main")
public class MainAuto extends MatchOpMode {
    /**
     * How far an AprilTag fix may disagree with the selected start pose before it is called out.
     *
     * <p>A large disagreement almost always means the wrong alliance or start position was picked,
     * not that the camera is wrong. Catching that in init costs nothing; catching it after the
     * routine has driven into the wrong half of the field costs the match.
     */
    public static double START_POSE_DISAGREEMENT_INCHES = 18.0;

    private final AutoSelector selector = new AutoSelector();

    private Alliance alliance = Alliance.RED;
    private StartPosition startPosition = StartPosition.LEFT;
    private AutoRoutine routine = null;

    @Override
    protected String logTag() {
        return "auto";
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.AUTONOMOUS;
    }

    @Override
    protected void onInit() {
        robot.intake.defaultIdleCommand().schedule();
    }

    @Override
    protected void onInitLoop() {
        selector.poll(gamepad1);
        alliance = selector.getAlliance();
        startPosition = selector.getStart();

        // Seed the pose from the chosen start position so the first path begins from the right
        // place. An AprilTag fix, if one is visible, is more trustworthy and overrides it.
        Pose start = FieldConstants.forAlliance(FieldConstants.startPose(startPosition), alliance);
        robot.drivetrain.setStartingPose(start);
        boolean sawTag = robot.tryLocalizeFromAprilTag();

        telemetry.addLine(selector.render());
        telemetry.addLine();
        if (!selector.isConfirmed()) {
            // The routine runs whether or not this was confirmed — refusing to move would be worse.
            // But an unconfirmed selector usually means nobody set the alliance, and a mirrored
            // routine driving the wrong way is not subtle.
            telemetry.addLine(">> NOT CONFIRMED - press A. Routine will use the values shown.");
        }
        telemetry.addData("Start pose", start);
        telemetry.addData("AprilTag fix?", sawTag ? "yes (overrides start pose)" : "none");

        double disagreement = startPoseDisagreement(start);
        if (!Double.isNaN(disagreement)) {
            telemetry.addData("Tag vs start pose", "%.1f in", disagreement);
            if (disagreement > START_POSE_DISAGREEMENT_INCHES) {
                telemetry.addLine(">> CHECK ALLIANCE/START - the camera says the robot is "
                        + "somewhere else entirely.");
            }
        }

        telemetry.addData("Pose", robot.drivetrain.getPose());
        Drawing.drawRobot(robot.drivetrain.getPose());
        Drawing.sendPacket();
    }

    /**
     * Distance in inches between the selected start pose and the current AprilTag fix, or
     * {@code NaN} when no trustworthy fix is available.
     */
    private double startPoseDisagreement(Pose selected) {
        Pose fromTag = robot.limelight.getBotposeAsPedroPose();
        if (fromTag == null || selected == null) return Double.NaN;
        return Math.hypot(fromTag.getX() - selected.getX(), fromTag.getY() - selected.getY());
    }

    @Override
    protected void onStart() {
        robot.poseFusion.seed(robot.drivetrain.getPose());
        routine = new AutoRoutine(robot, alliance);
        routine.build().schedule();
    }

    @Override
    protected void onAfterAct() {
        // Saved every loop rather than in stop(): if this OpMode is interrupted, teleop should still
        // inherit wherever the robot actually got to.
        PoseStorage.save(robot.drivetrain.getPose(), alliance, startPosition);
    }

    @Override
    protected void onTelemetry() {
        if (routine != null) {
            telemetry.addData("Leg", routine.getCurrentLeg());
            telemetry.addData("Missed legs", routine.getMissedLegs());
            for (String entry : routine.getLegLog()) telemetry.addLine("  " + entry);
            telemetry.addLine();
        }
        MatchClock clock = robot.getMatchClock();
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Loop", loopStats.getStatus());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Localization", robot.poseFusion.getStatus());
        telemetry.addData("Following path?", robot.drivetrain.isFollowingPath());
        telemetry.addData("hasPollen?", robot.intake.hasPollen());
    }

    @Override
    protected void onDraw() {
        Drawing.drawDebug(robot.drivetrain.getFollower());
    }

    @Override
    protected void onStop() {
        PoseStorage.save(robot.drivetrain.getPose(), alliance, startPosition);
    }
}
