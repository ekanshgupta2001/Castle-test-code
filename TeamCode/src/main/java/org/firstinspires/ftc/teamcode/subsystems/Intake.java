package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.control.JamDetector;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.BooleanSupplier;

/**
 * Single-motor intake driven by velocity control, with stall-triggered anti-jam.
 *
 * <p><b>The core pattern:</b> commands and buttons never touch the motor. They set
 * {@link #targetVelocity} and a {@link Mode}; {@link #update()} is the single place that writes to
 * hardware, once per loop. That keeps the motor under one authority no matter how many commands are
 * fighting, and makes the anti-jam logic possible — it can override the request on its way out.
 *
 * <p>Consequence worth knowing: calling {@code intake()} does nothing until the next
 * {@code update()}. Code that sets a velocity and immediately reads {@link #getVelocityTicksPerSec()}
 * will read the <em>previous</em> command's speed.
 */
@Configurable
public class Intake {
    /**
     * goBILDA 5203 series, 435 RPM (13.7:1 gearbox) = <b>384.5 ticks/rev</b>.
     * Free speed = 435 rev/min x 384.5 ticks/rev / 60 s = ~2787 ticks/sec.
     *
     * <p>Do not confuse this with the 312 RPM (19.2:1) motor, which is the 537.7 ticks/rev part.
     * Pairing 435 RPM with 537.7 ticks/rev overstates the ceiling by ~40% and makes every velocity
     * request unreachable — the PIDF then saturates at full power, current sits high, and the stall
     * detector below trips during perfectly normal intaking.
     *
     * <p>Verify with the SelfTest OpMode: command {@link #INTAKE_TICKS_PER_SEC} and read back the
     * actual velocity. If your motor is a different part, fix this number first.
     */
    public static double MOTOR_FREE_SPEED_TICKS_PER_SEC = 2787;

    /** ~90% of free speed. Leaves the velocity PIDF headroom to actually close its loop. */
    public static double INTAKE_TICKS_PER_SEC = 2500;
    public static double OUTTAKE_TICKS_PER_SEC = -1400;
    public static double EJECT_TICKS_PER_SEC = -2500;
    /** Gentle inward pressure to retain a captured piece. Deliberately well below stall current. */
    public static double HOLD_TICKS_PER_SEC = 200;

    public static double STALL_CURRENT_AMPS = 5.0;
    public static long STALL_TIMEOUT_MS = 200;
    public static double UNJAM_TICKS_PER_SEC = -2500;
    public static long UNJAM_DURATION_MS = 150;
    public static boolean ANTI_JAM_ENABLED = true;
    /** Consecutive unjam attempts before giving up, so a hard jam can't cook the motor all match. */
    public static int MAX_UNJAM_ATTEMPTS = 3;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    /** What the intake is being asked to do. Drives anti-jam eligibility — see {@link #update()}. */
    public enum Mode { IDLE, INTAKING, OUTTAKING, EJECTING, HOLDING }

    private final DcMotorEx motor;
    private final Clock clock;
    private double targetVelocity = 0;
    private Mode mode = Mode.IDLE;
    private boolean hasPiece = false;
    private BooleanSupplier capturedSupplier = () -> false;

    /** The stall/un-jam state machine. Lives in {@code util/} so it can be unit tested. */
    private final JamDetector jamDetector = new JamDetector();

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.INTAKE_MOTOR);
    }

    public Intake(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, Clock.system());
    }

    public Intake(HardwareMap hardwareMap, String name, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name), clock);
    }

    /**
     * Builds on an already-resolved motor, or {@code null} for "not fitted". Tests inject a fake
     * motor and a fake clock here; the hardware-map constructors above all end up in this one.
     */
    public Intake(DcMotorEx motor, Clock clock) {
        this.motor = motor;
        this.clock = clock;
        if (motor == null) return;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** False when the motor is missing from the robot configuration. All calls then no-op. */
    public boolean isAvailable() {
        return motor != null;
    }

    public void setCapturedSupplier(BooleanSupplier supplier) {
        this.capturedSupplier = supplier == null ? () -> false : supplier;
    }

    /** Sets the raw velocity request. Prefer the named modes below so anti-jam stays correct. */
    public void setVelocity(double ticksPerSec) {
        setMode(ticksPerSec > 0 ? Mode.INTAKING : ticksPerSec < 0 ? Mode.EJECTING : Mode.IDLE,
                ticksPerSec);
    }

    private void setMode(Mode newMode, double ticksPerSec) {
        // Any deliberate mode change abandons an in-progress unjam. Without this, re-pressing
        // intake within UNJAM_DURATION_MS would silently run the motor backwards instead.
        if (newMode != mode) {
            if (newMode == Mode.INTAKING) {
                // Re-entering intaking keeps the attempt count, so repeatedly mashing the button
                // cannot bypass MAX_UNJAM_ATTEMPTS and cook a motor against a hard jam.
                jamDetector.resetTiming();
            } else {
                jamDetector.reset();
            }
        }
        mode = newMode;
        targetVelocity = ticksPerSec;
    }

    public void intake() {
        setMode(Mode.INTAKING, INTAKE_TICKS_PER_SEC);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING, OUTTAKE_TICKS_PER_SEC);
        markEmpty();
    }

    public void eject() {
        setMode(Mode.EJECTING, EJECT_TICKS_PER_SEC);
        markEmpty();
    }

    public void hold() {
        setMode(Mode.HOLDING, HOLD_TICKS_PER_SEC);
    }

    public void stop() {
        setMode(Mode.IDLE, 0);
    }

    public Mode getMode() {
        return mode;
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    public void markCaptured() {
        hasPiece = true;
    }

    public void markEmpty() {
        hasPiece = false;
    }

    public double getCurrentAmps() {
        return motor == null ? 0 : motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getVelocityTicksPerSec() {
        return motor == null ? 0 : motor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * True while over-current has been observed but has not yet lasted {@link #STALL_TIMEOUT_MS}.
     * This is a <em>suspicion</em>, not a confirmed jam — during the actual unjam reversal this
     * reads false and {@link #isUnjamming()} reads true.
     */
    public boolean isStallSuspected() {
        return jamDetector.isStallSuspected();
    }

    public boolean isUnjamming() {
        return jamDetector.isUnjamming(clock.nowMs());
    }

    public int getUnjamAttempts() {
        return jamDetector.getAttempts();
    }

    /** True once anti-jam has exhausted {@link #MAX_UNJAM_ATTEMPTS} and stopped trying. */
    public boolean hasGivenUpUnjamming() {
        return jamDetector.hasGivenUp();
    }

    public void update() {
        if (motor == null) return;
        long now = clock.nowMs();

        if (mode == Mode.INTAKING && !hasPiece && capturedSupplier.getAsBoolean()) {
            hasPiece = true;
        }

        // Pushed in every loop so live dashboard edits reach the detector.
        jamDetector.configure(
                STALL_CURRENT_AMPS, STALL_TIMEOUT_MS, UNJAM_DURATION_MS, MAX_UNJAM_ATTEMPTS);

        // Anti-jam applies to INTAKING only. It must NOT apply to HOLDING: holding a captured piece
        // against a hard stop is, by definition, a stalled motor — so a check that ran there would
        // see the hold current, "unjam", and spit the piece straight back out.
        boolean antiJamEligible = ANTI_JAM_ENABLED && mode == Mode.INTAKING;

        if (jamDetector.update(now, antiJamEligible, motor.getCurrent(CurrentUnit.AMPS))) {
            motor.setVelocity(UNJAM_TICKS_PER_SEC);
            return;
        }

        motor.setVelocity(targetVelocity);
    }

    public Command intakeCommand() {
        return Command.build()
                .setStart(this::intake)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command outtakeCommand() {
        return Command.build()
                .setStart(this::outtake)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command ejectCommand() {
        return Command.build()
                .setStart(this::eject)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command holdCommand() {
        return Command.build()
                .setStart(this::hold)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command stopCommand() {
        return Command.build()
                .setStart(this::stop)
                .setDone(() -> true)
                .requiring(this);
    }

    public Command runForMs(double ticksPerSec, long ms) {
        Command run = Command.build()
                .setStart(() -> setVelocity(ticksPerSec))
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
        return race(run, waitMs(ms));
    }


    public Command captureCommand(BooleanSupplier captured) {
        return Command.build()
                .setStart(this::intake)
                .setDone(captured)
                .setEnd(ec -> {
                    if (ec == EndCondition.NATURALLY) markCaptured();
                    stop();
                })
                .requiring(this);
    }

    public Command captureAndHoldCommand() {
        return captureAndHoldCommand(() -> capturedSupplier.getAsBoolean());
    }

    public Command captureAndHoldCommand(BooleanSupplier captured) {
        return sequential(captureCommand(captured), holdCommand());
    }

    /**
     * Schedule once at OpMode init. Suspends when a real intake command takes the resource and
     * resumes when that command ends, holding gently when carrying a piece and idling otherwise.
     *
     * <p>The behaviour lives in {@code setExecute}, not {@code setStart}, on purpose: the Scheduler's
     * resume path re-adds the command to the running set without re-calling {@code start()}, so
     * start-only logic would never run again after the first suspension.
     *
     * <p>{@link BlockedBehavior#QUEUE} matters too. At priority -1 the default {@code CANCEL} would
     * drop this permanently if anything were already holding the intake when it was scheduled.
     */
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(() -> {
                    if (hasPiece) hold();
                    else stop();
                })
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }
}
