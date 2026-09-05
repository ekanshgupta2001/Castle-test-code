package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.AutoSelector;
import org.firstinspires.ftc.teamcode.util.MatchClock;
import org.firstinspires.ftc.teamcode.util.diagnostics.Drawing;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

import java.util.ArrayList;
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
 * <h2>Every leg is bounded, checked, and recoverable</h2>
 * A leg is not simply "follow this path". It is:
 * <ol>
 *   <li><b>Bounded.</b> {@code race(drive, waitMs(budget))}. A follower pinned against a wall or a
 *       defending robot keeps {@code isBusy()} true forever, and without a budget the enclosing
 *       sequential blocks for the rest of the period — burning autonomous with no indication why.</li>
 *   <li><b>Checked.</b> On finishing, the leg asks the follower whether the robot is actually near
 *       the target pose. A leg that ran out of time is recorded as MISSED rather than assumed good,
 *       which is what makes the branch below possible at all.</li>
 *   <li><b>Planned from the present.</b> Each path is built at command start from the robot's
 *       <em>current</em> pose, not from where the previous leg was supposed to end. After a missed
 *       leg that is the difference between recovering and driving a path that begins somewhere the
 *       robot never reached.</li>
 * </ol>
 *
 * <p>If any leg is missed, the scoring block is skipped and the robot goes straight to park — a
 * parked robot that scored nothing beats a robot stuck against a wall, and both beat a routine that
 * silently stopped executing. Park is attempted unconditionally for the same reason.
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
@Configurable
@Autonomous(name = "Auto", group = "Main")
public class MainAuto extends MatchOpMode {
    /**
     * Time budget for a normal driving leg.
     *
     * <p>Set this generously — it is a failsafe, not a schedule. A budget tighter than the leg's
     * real duration converts every slow-but-fine run into a reported failure and skips the scoring
     * block for no reason.
     */
    public static long LEG_TIMEOUT_MS = 5000;
    /** Park gets its own, shorter budget: whatever is left is better spent stopped than driving. */
    public static long PARK_TIMEOUT_MS = 3500;
    /** How long to run the intake backwards to release a game piece. */
    public static long SCORE_EJECT_MS = 600;
    /** How close counts as having arrived, in inches, on each axis. */
    public static double ARRIVAL_TOLERANCE_INCHES = 4.0;
    /** Fraction along leg one at which the intake spins up, rather than waiting for arrival. */
    public static double INTAKE_SPINUP_T = 0.6;
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

    /** Per-leg results, in order, for telemetry and the post-match log. */
    private final List<String> legLog = new ArrayList<>();
    private String currentLeg = "none";
    private int missedLegs = 0;


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
        buildRoutine().schedule();
    }

    @Override
    protected void onAfterAct() {
        // Saved every loop rather than in stop(): if this OpMode is interrupted, teleop should still
        // inherit wherever the robot actually got to.
        PoseStorage.save(robot.drivetrain.getPose(), alliance, startPosition);
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Leg", currentLeg);
        telemetry.addData("Missed legs", missedLegs);
        for (String entry : legLog) telemetry.addLine("  " + entry);
        telemetry.addLine();
        telemetry.addData("Time", robot.getMatchClock().getStatus());
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

    /**
     * The routine. Poses come from {@link FieldConstants} and are mirrored for the alliance, so
     * there is exactly one copy of each location.
     */
    private Command buildRoutine() {
        Pose staging = alliancePose(FieldConstants.BLUE_STAGING);
        Pose score = alliancePose(FieldConstants.BLUE_SCORE);
        Pose park = alliancePose(FieldConstants.BLUE_PARK);

        return sequential(
                // Leg 1: drive to staging, spinning the intake up part way rather than on arrival.
                leg("staging", staging, true, LEG_TIMEOUT_MS, robot.intake::intake),

                // Scoring only makes sense if we actually reached staging. If we did not, this
                // whole block is skipped and the routine falls through to park.
                skipIfAnyLegMissed(sequential(
                        leg("score", score, true, LEG_TIMEOUT_MS),
                        robot.intake.runForMs(Intake.OUTTAKE_TICKS_PER_SEC, SCORE_EJECT_MS),
                        // runForMs sets a negative velocity, which is not the same as declaring the
                        // intake empty - without this the robot still believes it is carrying.
                        instant(robot.intake::markEmpty)
                )),

                // Park is attempted no matter what happened above, and is planned from wherever the
                // robot actually ended up.
                leg("park", park, false, PARK_TIMEOUT_MS),
                instant(robot.intake::stop)
        );
    }

    /**
     * Skips {@code command} when an earlier leg was missed.
     *
     * <p>Deliberately {@code conditional(...)} and not {@code command.unless(...)}. Ivy implements
     * {@code unless} as {@code conditional(condition, Command.NOOP, this)}, and {@code NOOP} is
     * built with the default done-supplier {@code () -> false} — so on the skip path it never
     * finishes and the enclosing sequential hangs for the rest of the match. {@code instant()} sets
     * {@code done} to true and completes on its first tick.
     */
    private Command skipIfAnyLegMissed(Command command) {
        return conditional(() -> missedLegs == 0, command, instant(() -> { }));
    }

    private Command leg(String name, Pose target, boolean holdEnd, long budgetMs) {
        return leg(name, target, holdEnd, budgetMs, null);
    }

    /**
     * One bounded, verified driving leg.
     *
     * @param name     shown in telemetry and recorded in the leg log
     * @param target   where this leg is trying to get to
     * @param holdEnd  whether to keep station-keeping at the end pose
     * @param budgetMs failsafe timeout; exceeding it marks the leg MISSED and moves on
     * @param onApproach optional action fired {@link #INTAKE_SPINUP_T} of the way along, or null
     */
    private Command leg(String name, Pose target, boolean holdEnd, long budgetMs,
                        Runnable onApproach) {
        return sequential(
                instant(() -> currentLeg = name),
                race(
                        // Built lazily so the path starts from where the robot is now. A stored
                        // PathChain would also cache its endpoints and re-drive a stale target.
                        robot.drivetrain.followLazyCommand(
                                () -> pathToward(target, onApproach), holdEnd),
                        waitMs(budgetMs)
                ),
                instant(() -> finishLeg(name, target))
        );
    }

    /** Records whether the leg actually arrived. This is what the skip logic reads. */
    private void finishLeg(String name, Pose target) {
        boolean arrived = arrivedAt(target);
        if (!arrived) missedLegs++;
        legLog.add(name + (arrived ? " : ok" : " : MISSED"));
        currentLeg = arrived ? "done " + name : "MISSED " + name;
    }

    /**
     * Whether the robot is within {@link #ARRIVAL_TOLERANCE_INCHES} of a pose.
     *
     * <p>Returns false when there is no follower, which correctly reports every leg as missed on a
     * robot whose drivetrain failed to build — rather than claiming a routine ran perfectly while
     * standing still.
     */
    private boolean arrivedAt(Pose target) {
        Follower follower = robot.drivetrain.getFollower();
        if (follower == null || target == null) return false;
        return follower.atPose(target, ARRIVAL_TOLERANCE_INCHES, ARRIVAL_TOLERANCE_INCHES);
    }

    private Pose alliancePose(Pose bluePose) {
        return FieldConstants.forAlliance(bluePose, alliance);
    }

    /**
     * Builds a fresh straight-line path from the robot's current pose to {@code target}, or
     * {@code null} if the pose is unknown — in which case the leg finishes immediately and is
     * recorded as missed.
     */
    private PathChain pathToward(Pose target, Runnable onApproach) {
        Follower follower = robot.drivetrain.getFollower();
        if (follower == null || target == null) return null;

        Pose current = robot.drivetrain.getPose();
        if (current == null) return null;

        if (onApproach == null) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(current, target))
                    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                    .build();
        }
        return follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .addParametricCallback(INTAKE_SPINUP_T, onApproach)
                .build();
    }
}
