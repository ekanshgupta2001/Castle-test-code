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

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The autonomous routine as one Ivy command tree, separate from the OpMode that schedules it.
 *
 * <p>Split out of {@code MainAuto} so it can be built and run on the JVM: an {@code OpMode} needs
 * the SDK to instantiate, but this class needs only a {@link Robot}. {@code AutoRoutineTest} runs
 * the whole tree against fakes and checks the skip logic, the ownership declaration, and that a
 * missed leg still parks.
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
 * <h2>The routine owns the intake and the drivetrain for its whole duration</h2>
 * Declared explicitly at the end of {@link #build()} with {@code requiring(...)}. Ivy groups inherit
 * their children's requirements, so the group already picks up the drivetrain from every leg and
 * the intake from {@code runForMs} in the scoring block. That inheritance is what makes the direct
 * {@code robot.intake::intake} callback in leg one safe: while this routine holds the intake,
 * {@code Intake.defaultIdleCommand} is suspended and cannot overwrite the request. Delete the
 * scoring block and, without the explicit declaration, the idle command would come back and
 * silently stop the intake one loop after the callback. Declaring the ownership makes the
 * routine's behaviour independent of what happens to be inside it.
 *
 * <h2>Before this can run on a robot</h2>
 * {@code pedroPathing/Constants.java} is still default-constructed — no drivetrain, no localizer —
 * so the follower cannot move a real robot yet. The poses in {@link FieldConstants} are placeholders
 * too. Both must be filled in before this does anything useful on the field.
 */
@Configurable
public final class AutoRoutine {
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

    private final Robot robot;
    private final Alliance alliance;

    /** Per-leg results, in order, for telemetry and the post-match log. */
    private final List<String> legLog = new ArrayList<>();
    private String currentLeg = "none";
    private int missedLegs = 0;

    public AutoRoutine(Robot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;
    }

    /** Per-leg outcomes so far, e.g. {@code "staging : ok"}, {@code "score : MISSED"}. */
    public List<String> getLegLog() {
        return Collections.unmodifiableList(legLog);
    }

    /** The leg in progress, or the last leg's outcome once it has finished. */
    public String getCurrentLeg() {
        return currentLeg;
    }

    public int getMissedLegs() {
        return missedLegs;
    }

    /**
     * The routine. Poses come from {@link FieldConstants} and are mirrored for the alliance, so
     * there is exactly one copy of each location. Build it once, after the alliance is known.
     */
    public Command build() {
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
        ).requiring(robot.intake, robot.drivetrain);
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
     * @param name       shown in telemetry and recorded in the leg log
     * @param target     where this leg is trying to get to
     * @param holdEnd    whether to keep station-keeping at the end pose
     * @param budgetMs   failsafe timeout; exceeding it marks the leg MISSED and moves on
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
     * <p>False when there is no drivetrain, which correctly reports every leg as missed on a robot
     * whose drivetrain failed to build — rather than claiming a routine ran perfectly while
     * standing still.
     */
    private boolean arrivedAt(Pose target) {
        return robot.drivetrain.atPose(target, ARRIVAL_TOLERANCE_INCHES, ARRIVAL_TOLERANCE_INCHES);
    }

    private Pose alliancePose(Pose bluePose) {
        return FieldConstants.forAlliance(bluePose, alliance);
    }

    /**
     * Builds a fresh straight-line path from the robot's current pose to {@code target}, or
     * {@code null} if the pose or the follower is unknown — in which case the leg finishes
     * immediately and is recorded as missed unless the robot already happens to be there.
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
