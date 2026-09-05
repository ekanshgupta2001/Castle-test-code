package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * The intake's commands, run through Ivy's real {@link Scheduler} against a fake motor.
 *
 * <p>Each test steps the loop the way {@code MatchOpMode} does: scheduler, then the actuator
 * write. That ordering is the whole point of several of these tests.
 */
public class IntakeCommandTest {
    private static final double EPS = 1e-9;

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Intake intake;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        intake = new Intake(motor, clock);
        Intake.ANTI_JAM_ENABLED = true;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    /** One robot loop: decide, then act. */
    private void tick() {
        Scheduler.execute();
        intake.update();
        clock.advance(20);
    }

    @Test
    public void constructorConfiguresTheMotorWithoutMovingIt() {
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertEquals(0, motor.velocityWrites);
        assertTrue(intake.isAvailable());
    }

    @Test
    public void defaultIdleStopsTheMotorWhenEmpty() {
        intake.defaultIdleCommand().schedule();
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.IDLE, intake.getMode());
    }

    @Test
    public void defaultIdleHoldsGentlyWhenCarrying() {
        intake.markCaptured();
        intake.defaultIdleCommand().schedule();
        tick();
        assertEquals(Intake.HOLD_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.HOLDING, intake.getMode());
    }

    @Test
    public void intakeCommandPreemptsIdleAndIdleResumesWhenItEnds() {
        intake.defaultIdleCommand().schedule();
        tick();

        Command run = intake.intakeCommand();
        run.schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        Scheduler.cancel(run);
        tick();
        assertEquals("idle must come back on its own", 0, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.IDLE, intake.getMode());
    }

    /**
     * The default-command trap. An instant that does not require the intake runs its body at
     * schedule time, then the idle command's execute() overwrites the request before the write.
     * This is what {@code Concept: Commands} demo 9 shows on the robot.
     */
    @Test
    public void aDirectCallUnderTheDefaultCommandNeverReachesTheMotor() {
        intake.defaultIdleCommand().schedule();
        tick();

        instant(() -> intake.intake()).schedule();
        assertEquals("the instant ran immediately", Intake.Mode.INTAKING, intake.getMode());

        tick();
        assertEquals(Intake.Mode.IDLE, intake.getMode());
        assertEquals("the motor never saw the request", 0, motor.maxCommandedVelocity, EPS);
    }

    /** The exception to the trap, and the rule {@code AutoRoutine} relies on. */
    @Test
    public void aGroupThatRequiresTheIntakeMayCallItDirectly() {
        intake.defaultIdleCommand().schedule();
        tick();

        Command group = sequential(instant(intake::intake), waitMs(10_000)).requiring(intake);
        group.schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        tick();
        assertEquals("still ours while the group runs", Intake.INTAKE_TICKS_PER_SEC,
                motor.commandedVelocity, EPS);
    }

    @Test
    public void captureCommandMarksCapturedOnlyWhenItFinishesNaturally() {
        boolean[] seen = {false};
        Command capture = intake.captureCommand(() -> seen[0]);
        capture.schedule();
        tick();
        assertFalse(intake.hasPollen());
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        seen[0] = true;
        tick();
        assertTrue(intake.hasPollen());
        assertEquals(0, motor.commandedVelocity, EPS);
        assertFalse(Scheduler.isScheduled(capture));
    }

    @Test
    public void anInterruptedCaptureDoesNotClaimAPiece() {
        Command capture = intake.captureCommand(() -> false);
        capture.schedule();
        tick();
        Scheduler.cancel(capture);
        tick();
        assertFalse(intake.hasPollen());
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void updateLatchesPossessionFromTheSupplierWhileIntaking() {
        boolean[] atSensor = {false};
        intake.setCapturedSupplier(() -> atSensor[0]);
        intake.intakeCommand().schedule();
        tick();
        assertFalse(intake.hasPollen());
        atSensor[0] = true;
        tick();
        assertTrue(intake.hasPollen());
    }

    @Test
    public void sustainedOverCurrentWhileIntakingReversesTheMotor() {
        intake.intakeCommand().schedule();
        motor.currentAmps = Intake.STALL_CURRENT_AMPS + 3;

        tick();                                                     // t = 0: stall first seen
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        while (clock.nowMs() < Intake.STALL_TIMEOUT_MS) tick();    // inside the dwell
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        tick();                                                     // dwell elapsed
        assertEquals(Intake.UNJAM_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertTrue(intake.isUnjamming());
        assertEquals(1, intake.getUnjamAttempts());
    }

    @Test
    public void holdingAgainstAStopNeverTriggersAntiJam() {
        intake.markCaptured();
        intake.holdCommand().schedule();
        motor.currentAmps = 20;
        for (int i = 0; i < 50; i++) tick();
        assertEquals(Intake.HOLD_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(0, intake.getUnjamAttempts());
    }

    @Test
    public void anUnavailableIntakeNoOpsEverywhere() {
        Intake none = new Intake((DcMotorEx) null, clock);
        assertFalse(none.isAvailable());
        none.intake();
        none.update();
        assertEquals(0, none.getVelocityTicksPerSec(), EPS);
        assertEquals(0, none.getCurrentAmps(), EPS);
    }
}
