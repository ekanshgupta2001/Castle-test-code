package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Drivetrain commands, heading hold, and the hold-at-end semantics, against a fake follower.
 *
 * <p>The fake mirrors Pedro's state machine exactly ({@code isBusy} false while holding,
 * {@code startTeleopDrive} breaks a hold), because every one of these behaviours came from a
 * real mismatch between what the code assumed and what the follower does.
 */
public class DrivetrainCommandTest {
    private static final double EPS = 1e-9;

    private FakePathFollower fake;
    private FakeClock clock;
    private Drivetrain drivetrain;
    private double turnStick = 0;

    @Before
    public void setUp() {
        Scheduler.reset();
        fake = new FakePathFollower();
        clock = new FakeClock();
        drivetrain = new Drivetrain(fake, clock);
        Drivetrain.HEADING_HOLD_ENABLED = true;
        Drivetrain.MIN_PATH_MS = 60;
        turnStick = 0;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private static PathChain somePath() {
        return new PathChain(new Path(new BezierPoint(new Pose(12, 24, 0))));
    }

    private Command driverControl() {
        return drivetrain.driverControlCommand(() -> 0, () -> 0, () -> turnStick);
    }

    /** One robot loop: decide, then act. */
    private void tick() {
        Scheduler.execute();
        drivetrain.update();
        clock.advance(20);
    }

    // ---- Availability ----

    @Test
    public void unavailableDrivetrainCommandsFinishAtOnce() {
        Drivetrain none = new Drivetrain((PathFollower) null, clock);
        assertFalse(none.isAvailable());
        assertNull(none.getPose());
        Command follow = none.followLazyCommand(DrivetrainCommandTest::somePath, true);
        follow.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(follow));
        Command turn = none.turnToCommand(1.0);
        turn.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(turn));
    }

    @Test
    public void rawFollowerIsNullUnlessBuiltOnPedro() {
        assertNull(drivetrain.getFollower());
        assertTrue(drivetrain.isAvailable());
    }

    // ---- Lazy path command ----

    @Test
    public void nullSupplierFinishesImmediatelyWithoutTouchingTheFollower() {
        Command cmd = drivetrain.followLazyCommand(() -> null, true);
        cmd.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(0, fake.followPathCalls);
        assertEquals("no path started, so nothing to cancel", 0, fake.startTeleopDriveCalls);
    }

    @Test
    public void pathCannotFinishBeforeMinPathMs() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        assertEquals(1, fake.followPathCalls);
        fake.finishPath();                          // follower says done on the very first tick

        Scheduler.execute();
        assertTrue("must not believe a zero-time path", Scheduler.isScheduled(cmd));

        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
    }

    @Test
    public void naturalEndWithHoldEndLeavesTheFollowerHolding() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();

        assertFalse(Scheduler.isScheduled(cmd));
        assertTrue("still station-keeping", fake.holding);
        assertFalse(fake.teleop);
        assertEquals("cancelPath must not have run", 0, fake.startTeleopDriveCalls);
    }

    @Test
    public void naturalEndWithoutHoldEndReturnsControlToTeleop() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();

        assertFalse(Scheduler.isScheduled(cmd));
        assertTrue(fake.teleop);
        assertFalse(fake.holding);
        assertEquals(1, fake.startTeleopDriveCalls);
    }

    @Test
    public void interruptionAlwaysReturnsControlToTeleopEvenWhenAskedToHold() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();
        assertTrue(fake.busy);
        Scheduler.cancel(cmd);
        assertTrue("the follower would otherwise keep driving itself", fake.teleop);
        assertFalse(fake.busy);
        assertFalse(fake.holding);
    }

    @Test
    public void driverControlResumingReleasesAHeldPose() {
        driverControl().schedule();
        tick();

        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();                             // suspends driver control
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        tick();                                     // path ends; driver control is re-added
        assertTrue(fake.holding);

        tick();                                     // driver control executes again
        assertTrue("teleop re-engaged", fake.teleop);
        assertFalse("hold released by the driver taking over", fake.holding);
    }

    @Test
    public void followPathCommandHasTheSameGuardsAsTheLazyOne() {
        Command cmd = drivetrain.followPathCommand(somePath(), false);
        cmd.schedule();
        assertEquals(1, fake.followPathCalls);
        fake.finishPath();
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(cmd));
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertTrue(fake.teleop);
    }

    // ---- Teleop flag bookkeeping ----

    @Test
    public void cancelPathDoesNotMakeDriverControlTickTheFollowerASecondTime() {
        driverControl().schedule();
        tick();
        assertEquals(1, fake.startTeleopDriveCalls);

        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        Scheduler.cancel(cmd);                      // cancelPath -> startTeleopDrive
        assertEquals(2, fake.startTeleopDriveCalls);

        tick();                                     // driver control resumes
        tick();                                     // and executes
        assertEquals("already engaged; no extra startTeleopDrive", 2, fake.startTeleopDriveCalls);
    }

    @Test
    public void turnToCommandChangesNothingUntilItStarts() {
        Command turn = drivetrain.turnToCommand(1.0);
        assertNull("building a command must not touch the follower", fake.lastTurnTarget);

        turn.schedule();
        assertEquals(1.0, fake.lastTurnTarget, EPS);
        assertTrue(fake.busy);

        fake.finishTurn();
        Scheduler.execute();
        assertTrue("MIN_PATH_MS applies to turns too", Scheduler.isScheduled(turn));
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(turn));
        assertTrue("a snap hands the sticks back", fake.teleop);
    }

    @Test
    public void holdCommandHoldsUntilInterrupted() {
        Command hold = drivetrain.holdCommand();
        hold.schedule();
        assertTrue(fake.holding);
        for (int i = 0; i < 100; i++) tick();
        assertTrue("hold must not finish on its own", Scheduler.isScheduled(hold));

        Scheduler.cancel(hold);
        assertFalse(fake.holding);
        assertTrue(fake.teleop);
    }

    // ---- Heading hold ----

    @Test
    public void headingHoldCorrectsDriftAndYieldsToTheDriver() {
        fake.pose = new Pose(0, 0, 1.0);
        driverControl().schedule();
        tick();                                     // captures 1.0
        assertTrue(drivetrain.isHeadingHoldActive());
        assertEquals(1.0, drivetrain.getHeldHeading(), EPS);

        fake.pose = new Pose(0, 0, 1.2);            // knocked clockwise-positive by 0.2 rad
        tick();
        assertTrue("must correct back toward the held heading", fake.lastTurn < 0);
        assertTrue(Math.abs(fake.lastTurn) <= Drivetrain.HEADING_HOLD_MAX_TURN + EPS);

        turnStick = 0.5;                            // driver steers
        tick();
        assertEquals(0.5, fake.lastTurn, EPS);
        assertFalse("deliberate turn releases the hold", drivetrain.isHeadingHoldActive());
    }

    @Test
    public void resetHeadingReleasesTheHoldInsteadOfSpinningTheRobot() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());

        drivetrain.resetHeading();
        assertFalse(drivetrain.isHeadingHoldActive());
        assertEquals(0, fake.pose.getHeading(), EPS);

        tick();                                     // re-captures at 0
        tick();
        assertEquals("no correction toward the discarded heading", 0, fake.lastTurn, EPS);
    }

    @Test
    public void setPoseReleasesTheHold() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());
        drivetrain.setPose(new Pose(5, 5, 0.2));
        assertFalse(drivetrain.isHeadingHoldActive());
    }

    @Test
    public void suspendingDriverControlReleasesTheHoldAndZeroesTheSticks() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());

        drivetrain.holdCommand().schedule();       // preempts driver control
        assertFalse(drivetrain.isHeadingHoldActive());
        assertEquals(0, fake.lastForward, EPS);
        assertEquals(0, fake.lastTurn, EPS);
    }
}
