package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;

/**
 * The autonomous command tree, run to completion on the JVM.
 *
 * <p>There is no Pedro follower here, so every path is {@code null} and every leg finishes at once;
 * whether a leg counts as arrived is decided purely by the fake follower's pose against
 * {@link AutoRoutine#ARRIVAL_TOLERANCE_INCHES}. That isolates exactly the logic this class owns:
 * the skip, the ownership, the park-regardless rule.
 */
public class AutoRoutineTest {
    private static final long WALL_LIMIT_MS = 3000;

    private FakePathFollower follower;
    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Robot robot;

    private double savedTolerance;
    private long savedEject;

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        savedTolerance = AutoRoutine.ARRIVAL_TOLERANCE_INCHES;
        savedEject = AutoRoutine.SCORE_EJECT_MS;
        AutoRoutine.SCORE_EJECT_MS = 20;

        follower = new FakePathFollower();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        robot = new Robot(
                new Drivetrain(follower, clock),
                new Limelight(null),
                new ColorSensor(null),
                new Intake(motor, clock),
                new ExampleLift(null, clock),
                clock);
        robot.intake.defaultIdleCommand().schedule();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        AutoRoutine.ARRIVAL_TOLERANCE_INCHES = savedTolerance;
        AutoRoutine.SCORE_EJECT_MS = savedEject;
    }

    private void tick() throws InterruptedException {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
        Thread.sleep(2);
    }

    private void runToCompletion(Command routine) throws InterruptedException {
        long start = System.currentTimeMillis();
        routine.schedule();
        while (Scheduler.isScheduled(routine)) {
            tick();
            assertTrue("routine did not finish", System.currentTimeMillis() - start < WALL_LIMIT_MS);
        }
    }

    @Test
    public void routineDeclaresOwnershipOfTheIntakeAndDrivetrain() {
        Command routine = new AutoRoutine(robot, Alliance.BLUE).build();
        assertTrue(routine.requirements().contains(robot.intake));
        assertTrue(routine.requirements().contains(robot.drivetrain));
    }

    @Test
    public void whileTheRoutineRunsADirectIntakeCallIsNotOverridden() throws InterruptedException {
        tick();                                          // idle command is running
        Command routine = new AutoRoutine(robot, Alliance.BLUE).build();
        routine.schedule();                              // suspends the idle command

        robot.intake.intake();                           // what the mid-path callback does
        tick();
        assertEquals("the routine owns the intake, so the request survives",
                Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, 1e-9);
    }

    @Test
    public void aMissedLegSkipsScoringAndStillParks() throws InterruptedException {
        AutoRoutine.ARRIVAL_TOLERANCE_INCHES = 0;       // nothing ever counts as arrived
        robot.intake.markCaptured();
        AutoRoutine auto = new AutoRoutine(robot, Alliance.BLUE);
        runToCompletion(auto.build());

        assertEquals(Arrays.asList("staging : MISSED", "park : MISSED"), auto.getLegLog());
        assertEquals(2, auto.getMissedLegs());
        assertEquals("the score block never ran", 0, motor.minCommandedVelocity, 1e-9);
        assertTrue("still believed to be carrying", robot.intake.hasPollen());
        assertEquals("MISSED park", auto.getCurrentLeg());
    }

    @Test
    public void arrivingEverywhereRunsTheScoringBlockThenParks() throws InterruptedException {
        AutoRoutine.ARRIVAL_TOLERANCE_INCHES = 1e6;     // everything counts as arrived
        robot.intake.markCaptured();
        AutoRoutine auto = new AutoRoutine(robot, Alliance.BLUE);
        runToCompletion(auto.build());

        assertEquals(Arrays.asList("staging : ok", "score : ok", "park : ok"), auto.getLegLog());
        assertEquals(0, auto.getMissedLegs());
        assertEquals("ejected", Intake.OUTTAKE_TICKS_PER_SEC, motor.minCommandedVelocity, 1e-9);
        assertFalse("markEmpty ran after the eject", robot.intake.hasPollen());
    }

    @Test
    public void theIdleCommandComesBackWhenTheRoutineEnds() throws InterruptedException {
        AutoRoutine.ARRIVAL_TOLERANCE_INCHES = 0;
        runToCompletion(new AutoRoutine(robot, Alliance.BLUE).build());
        robot.intake.intake();                           // no owner now: idle must win again
        tick();
        assertEquals(Intake.Mode.IDLE, robot.intake.getMode());
        assertEquals(0, motor.commandedVelocity, 1e-9);
    }
}
