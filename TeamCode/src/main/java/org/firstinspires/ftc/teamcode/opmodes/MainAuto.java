package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSelector;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.FieldConstants;
import org.firstinspires.ftc.teamcode.util.MatchLogger;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.StartPosition;

import java.io.IOException;
import java.util.List;

/**
 * Autonomous routine.
 *
 * <h2>Shape of the thing</h2>
 * Init builds nothing but the selector. The routine itself is assembled in {@code start()}, after
 * the alliance is known, as one Ivy {@code sequential} of path legs and subsystem actions. The loop
 * then does nothing but tick sensors, the scheduler, and the follower — all the sequencing lives in
 * the command tree rather than in a hand-written state machine.
 *
 * <h2>Two things this demonstrates beyond "drive somewhere"</h2>
 * <ul>
 *   <li><b>Mid-path callbacks.</b> {@code addParametricCallback(t, action)} fires a mechanism part
 *       way along a leg, so the intake spins up before arrival instead of after it. Overlapping
 *       mechanism motion with driving is where autonomous cycle time actually comes from.</li>
 *   <li><b>Continuous handoff.</b> {@link PoseStorage} is written every loop, not once at the end,
 *       so teleop still inherits a good pose if this OpMode is stopped early.</li>
 * </ul>
 *
 * <h2>Before this can run</h2>
 * {@code pedroPathing/Constants.java} is still default-constructed — no drivetrain, no localizer —
 * so the follower cannot move a real robot yet. The poses in {@link FieldConstants} are placeholders
 * too. Both must be filled in before this does anything useful on the field.
 */
@Autonomous(name = "Auto", group = "Main")
public class MainAuto extends OpMode {
    private Robot robot;
    private MatchLogger logger;
    private final AutoSelector selector = new AutoSelector();

    private Alliance alliance = Alliance.RED;
    private StartPosition startPosition = StartPosition.LEFT;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        robot.intake.defaultIdleCommand().schedule();

        try {
            logger = new MatchLogger("auto");
        } catch (IOException e) {
            logger = null;
        }

        Drawing.init();

        List<String> missing = robot.getMissingHardware();
        if (!missing.isEmpty()) {
            telemetry.addLine("MISSING HARDWARE:");
            for (String m : missing) telemetry.addLine("  - " + m);
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        selector.poll(gamepad1);
        alliance = selector.getAlliance();
        startPosition = selector.getStart();

        robot.readSensors();

        // Seed the pose from the chosen start position so the first path begins from the right
        // place. An AprilTag fix, if one is visible, is more trustworthy and overrides it.
        Pose start = FieldConstants.forAlliance(FieldConstants.startPose(startPosition), alliance);
        robot.drivetrain.setStartingPose(start);
        boolean sawTag = robot.tryLocalizeFromAprilTag();

        telemetry.addLine(selector.render());
        telemetry.addLine();
        telemetry.addData("Start pose", start);
        telemetry.addData("AprilTag fix?", sawTag ? "yes (overrides start pose)" : "none");
        telemetry.addData("Pose", robot.drivetrain.getPose());
        Drawing.drawRobot(robot.drivetrain.getPose());
        Drawing.sendPacket();
    }

    @Override
    public void start() {
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.poseFusion.seed(robot.drivetrain.getPose());
        buildRoutine().schedule();
    }

    @Override
    public void loop() {
        robot.readSensors();
        robot.updateLocalization();
        Scheduler.execute();
        robot.writeActuators();

        // Saved every loop rather than in stop(): if this OpMode is interrupted, teleop should still
        // inherit wherever the robot actually got to.
        PoseStorage.save(robot.drivetrain.getPose(), alliance, startPosition);

        if (logger != null) logger.logRow(robot);

        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Localization", robot.poseFusion.getStatus());
        telemetry.addData("Following path?", robot.drivetrain.isFollowingPath());
        telemetry.addData("hasPollen?", robot.intake.hasPollen());
        Drawing.drawDebug(robot.drivetrain.getFollower());
    }

    @Override
    public void stop() {
        PoseStorage.save(robot.drivetrain.getPose(), alliance, startPosition);
        Scheduler.reset();
        if (robot != null) robot.stop();
        if (logger != null) logger.close();
    }

    /**
     * The routine. Poses come from {@link FieldConstants} and are mirrored for the alliance, so
     * there is exactly one copy of each location.
     */
    private com.pedropathing.ivy.Command buildRoutine() {
        Pose start = alliancePose(FieldConstants.startPose(startPosition));
        Pose staging = alliancePose(FieldConstants.BLUE_STAGING);
        Pose score = alliancePose(FieldConstants.BLUE_SCORE);
        Pose park = alliancePose(FieldConstants.BLUE_PARK);

        return sequential(
                // Leg 1: drive to staging, spinning the intake up 60% of the way there rather than
                // waiting until arrival.
                robot.drivetrain.followPathCommand(
                        legWithCallback(start, staging, 0.6, robot.intake::intake), true),

                // Leg 2: carry it to the scoring position, holding at the end so contact does not
                // push the robot off its mark.
                robot.drivetrain.followPathCommand(leg(staging, score), true),
                robot.intake.runForMs(-1400, 600),

                // Leg 3: park.
                robot.drivetrain.followPathCommand(leg(score, park), true),
                instant(robot.intake::stop)
        );
    }

    private Pose alliancePose(Pose bluePose) {
        return FieldConstants.forAlliance(bluePose, alliance);
    }

    private PathChain leg(Pose from, Pose to) {
        return robot.drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    /** A leg that fires {@code action} once the robot is {@code t} of the way along it. */
    private PathChain legWithCallback(Pose from, Pose to, double t, Runnable action) {
        return robot.drivetrain.getFollower().pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .addParametricCallback(t, action)
                .build();
    }
}
