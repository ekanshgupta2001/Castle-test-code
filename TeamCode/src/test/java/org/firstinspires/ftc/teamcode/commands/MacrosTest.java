package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.FakePathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.templates.ExampleLift;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Macros run end to end through Ivy against a fake follower, a fake motor, and no camera.
 *
 * <p>Ivy's {@code waitMs} reads the wall clock, so the macro timeouts are shortened to a few
 * milliseconds and the loop is driven with short real sleeps. Everything else is deterministic.
 */
public class MacrosTest {
    private static final long WALL_LIMIT_MS = 3000;

    private FakePathFollower follower;
    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Robot robot;

    private long savedWarmup, savedSearch, savedApproach, savedAlign, savedRelocalize, savedSnap;

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        savedWarmup = Macros.PIPELINE_WARMUP_MS;
        savedSearch = Macros.SEARCH_TIMEOUT_MS;
        savedApproach = Macros.APPROACH_TIMEOUT_MS;
        savedAlign = Macros.ALIGN_TIMEOUT_MS;
        savedRelocalize = Macros.RELOCALIZE_TIMEOUT_MS;
        savedSnap = Macros.SNAP_TIMEOUT_MS;
        Macros.PIPELINE_WARMUP_MS = 0;
        Macros.SEARCH_TIMEOUT_MS = 30;
        Macros.APPROACH_TIMEOUT_MS = 30;
        Macros.ALIGN_TIMEOUT_MS = 30;
        Macros.RELOCALIZE_TIMEOUT_MS = 30;
        Macros.SNAP_TIMEOUT_MS = 30;

        follower = new FakePathFollower();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        robot = new Robot(
                new Drivetrain(follower, clock),
                new Limelight(null),                // no camera in the "config"
                new ColorSensor(null),
                new Intake(motor, clock),
                new ExampleLift(null, clock),
                clock);
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Macros.PIPELINE_WARMUP_MS = savedWarmup;
        Macros.SEARCH_TIMEOUT_MS = savedSearch;
        Macros.APPROACH_TIMEOUT_MS = savedApproach;
        Macros.ALIGN_TIMEOUT_MS = savedAlign;
        Macros.RELOCALIZE_TIMEOUT_MS = savedRelocalize;
        Macros.SNAP_TIMEOUT_MS = savedSnap;
    }

    /** One robot loop, with a short real sleep so Ivy's wall-clock waits can elapse. */
    private void tick() throws InterruptedException {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
        Thread.sleep(2);
    }

    private void runToCompletion(Command macro) throws InterruptedException {
        runToCompletion(macro, null);
    }

    private void runToCompletion(Command macro, Runnable eachLoop) throws InterruptedException {
        long start = System.currentTimeMillis();
        macro.schedule();
        while (Scheduler.isScheduled(macro)) {
            if (eachLoop != null) eachLoop.run();
            tick();
            assertTrue("macro did not finish within " + WALL_LIMIT_MS + " ms: "
                    + robot.macros.getStatus(), System.currentTimeMillis() - start < WALL_LIMIT_MS);
        }
    }

    @Test
    public void fixturesAreWiredAsIntended() {
        assertFalse(robot.limelight.isAvailable());
        assertTrue(robot.drivetrain.isAvailable());
        assertTrue(robot.intake.isAvailable());
        assertFalse(robot.lift.isAvailable());
        assertEquals(Macros.Outcome.IDLE, robot.macros.getOutcome());
    }

    @Test
    public void collectPieceTimesOutWhenNothingIsVisible() throws InterruptedException {
        runToCompletion(robot.macros.collectPiece());
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals("no target, so no path", 0, follower.followPathCalls);
        assertFalse(follower.busy);
        assertFalse(robot.intake.hasPiece());
    }

    @Test
    public void collectPieceRequiresBothTheDrivetrainAndTheIntake() {
        Command macro = robot.macros.collectPiece();
        assertTrue(macro.requirements().contains(robot.drivetrain));
        assertTrue(macro.requirements().contains(robot.intake));
    }

    @Test
    public void relocalizeRequiresTheDrivetrain() {
        assertTrue(robot.macros.relocalize().requirements().contains(robot.drivetrain));
    }

    @Test
    public void relocalizeReportsNoTargetWithoutACamera() throws InterruptedException {
        runToCompletion(robot.macros.relocalize());
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
        assertEquals("pose must be untouched", 0, follower.setPoseCalls);
    }

    @Test
    public void servoAlignTimesOutWithoutATargetAndStopsTurning() throws InterruptedException {
        runToCompletion(robot.macros.servoAlignToPiece());
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals(0, follower.lastTurn, 1e-9);
    }

    @Test
    public void snapToHeadingSucceedsWhenTheTurnArrives() throws InterruptedException {
        Macros.SNAP_TIMEOUT_MS = 2000;
        double target = Math.PI / 2;
        runToCompletion(robot.macros.snapToHeading(target), () -> {
            if (follower.busy && follower.lastTurnTarget != null) follower.finishTurn();
        });
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(target, follower.lastTurnTarget, 1e-9);
        assertTrue("sticks handed back after the snap", follower.teleop);
    }

    @Test
    public void snapToHeadingTimesOutWhenTheTurnNeverArrives() throws InterruptedException {
        runToCompletion(robot.macros.snapToHeading(Math.PI));
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertTrue("the interrupted turn must release the follower", follower.teleop);
        assertFalse(follower.busy);
    }

    @Test
    public void abortMarksTheMacroCancelledAndStopsTheFollower() throws InterruptedException {
        Macros.SEARCH_TIMEOUT_MS = 5000;
        Command macro = robot.macros.collectPiece();
        macro.schedule();
        tick();
        tick();
        assertEquals(Macros.Outcome.RUNNING, robot.macros.getOutcome());

        Scheduler.cancel(macro);
        robot.abortMacro();
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertTrue(follower.teleop);
    }
}
