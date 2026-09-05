package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Teaching OpMode: how the Ivy command system works, one button at a time.
 *
 * <p>Run it, press each button, and watch the telemetry. Every binding below demonstrates exactly
 * one idea, in the order they are worth learning. Read the code alongside the robot.
 *
 * <p>Read {@code docs/03-commands.md} first if any of this is unfamiliar.
 *
 * <h2>Why every demo that moves the intake uses an {@code Intake} command factory</h2>
 * The intake's default idle command owns the motor whenever nothing else does, and it re-asserts
 * "stop" on every scheduler tick. A plain {@code robot.intake.intake()} call from a command that
 * does not {@code requiring(intake)} is therefore overwritten before the motor ever sees it.
 * {@code runForMs}, {@code intakeCommand} and friends all require the intake, which suspends the
 * idle command for as long as they run. Demo 9 triggers the failure on purpose so you can watch
 * it not happen.
 */
@TeleOp(name = "Concept: Commands", group = "Concept")
public class ConceptCommands extends OpMode {
    /** Short bursts, so one demo is over before the next button press. */
    private static final long BURST_MS = 500;

    private Robot robot;
    private String lastAction = "none";
    private int instantCount = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Ivy's Scheduler is static, so its state survives between OpMode runs. Reset it first or
        // you inherit whatever the previous OpMode left running.
        Scheduler.reset();

        // A default command: lowest priority, SUSPEND when preempted. It runs whenever nothing else
        // wants the intake, and comes back automatically afterwards. Schedule it exactly once.
        robot.intake.defaultIdleCommand().schedule();

        telemetry.addLine("Concept: Commands — press START, then try the buttons.");
        telemetry.update();
    }

    @Override
    public void start() {
        // *WasPressed() latches until read. init_loop() never reads them, so without this every
        // button bumped during init fires at once on the first loop.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
    }

    @Override
    public void loop() {
        robot.readSensors();      // 1. observe
        handleButtons();          // 2. decide
        Scheduler.execute();      //    run every scheduled command exactly once
        robot.writeActuators();   // 3. act

        telemetry.addLine("A = instant          B = wait 1s then run");
        telemetry.addLine("X = sequential       Y = parallel");
        telemetry.addLine("LB = race            RB = deadline");
        telemetry.addLine("DPAD-UP = conditional   DPAD-LEFT = conditional skip");
        telemetry.addLine("DPAD-RIGHT = direct call (watch it NOT move)");
        telemetry.addLine("DPAD-DOWN = cancel all");
        telemetry.addLine();
        telemetry.addData("Last scheduled", lastAction);
        telemetry.addData("Instant count", instantCount);
        telemetry.addData("Intake mode", robot.intake.getMode());
        telemetry.addData("hasPollen?", robot.intake.hasPollen());
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (robot != null) robot.stop();
    }

    private void handleButtons() {
        // ---- 1. instant: runs once, finishes immediately ----
        // Nothing to wait for, so it is done the same tick it starts. Good for flipping a flag or
        // recording a value; not a way to drive a mechanism that something else owns (see demo 9).
        if (gamepad1.aWasPressed()) {
            run("instant", instant(() -> instantCount++));
        }

        // ---- 2. sequential with a wait: steps run strictly in order ----
        if (gamepad1.bWasPressed()) {
            run("waitMs then intake", sequential(
                    waitMs(1000),
                    robot.intake.runForMs(Intake.INTAKE_TICKS_PER_SEC, BURST_MS)
            ));
        }

        // ---- 3. sequential: each step waits for the previous one to finish ----
        // runForMs stops the motor itself when its time is up, so no trailing "stop" is needed.
        if (gamepad1.xWasPressed()) {
            run("sequential", sequential(
                    robot.intake.runForMs(Intake.INTAKE_TICKS_PER_SEC, 400),
                    waitMs(200),
                    robot.intake.runForMs(Intake.OUTTAKE_TICKS_PER_SEC, 400)
            ));
        }

        // ---- 4. parallel: everything runs at once, group ends when ALL are done ----
        // Note both children here touch different things. Two commands requiring the SAME
        // subsystem cannot run in parallel — Ivy will resolve the conflict, not run both.
        if (gamepad1.yWasPressed()) {
            run("parallel", parallel(
                    robot.intake.runForMs(Intake.INTAKE_TICKS_PER_SEC, 800),
                    sequential(waitMs(300), instant(() -> lastAction = "parallel: half way"))
            ));
        }

        // ---- 5. race: ends as soon as the FIRST child finishes ----
        // This is how every timeout in this codebase is built: race(work, waitMs(limit)).
        if (gamepad1.leftBumperWasPressed()) {
            run("race (2s cap)", race(
                    waitUntil(() -> robot.intake.hasPollen()),
                    waitMs(2000)
            ));
        }

        // ---- 6. deadline: the FIRST child decides when the group ends ----
        // The others are cancelled the moment it finishes. Use it for "do X while Y happens".
        if (gamepad1.rightBumperWasPressed()) {
            run("deadline", deadline(
                    waitMs(1200),                       // the timekeeper
                    robot.intake.intakeCommand()        // cancelled when the timer expires
            ));
        }

        // ---- 7. conditional: the branch is chosen when the command STARTS, not when it is built ----
        // Both branches' requirements are claimed up front, so whichever runs owns the intake.
        if (gamepad1.dpadUpWasPressed()) {
            run("conditional", conditional(
                    () -> robot.intake.hasPollen(),
                    robot.intake.runForMs(Intake.EJECT_TICKS_PER_SEC, BURST_MS),
                    robot.intake.runForMs(Intake.INTAKE_TICKS_PER_SEC, BURST_MS)));
        }

        // ---- 8. the unless() trap ----
        // Ivy gives every command a decorator that LOOKS like the way to skip work:
        //
        //     command.unless(() -> shouldSkip)
        //
        // Do not use it inside a group. It is conditional(cond, Command.NOOP, this), and NOOP is
        // Command.build() with no setDone - so it inherits Ivy's default done-supplier of
        // () -> false and NEVER FINISHES. Taking the skip path hangs the enclosing sequential for
        // the rest of the match.
        //
        // Skip with an explicit no-op instead. instant() sets done to true, so it completes on its
        // first tick. AutoRoutine.skipIfAnyLegMissed is the real use of this.
        if (gamepad1.dpadLeftWasPressed()) {
            run("conditional skip", conditional(
                    () -> robot.intake.hasPollen(),
                    instant(() -> lastAction = "skipped: already carrying"),
                    robot.intake.runForMs(Intake.INTAKE_TICKS_PER_SEC, 800)));
        }

        // ---- 9. the default-command trap: a direct call that goes nowhere ----
        // This instant does not require the intake, so the idle command keeps running. The
        // Scheduler starts a newly scheduled command immediately, so intake() is set right here;
        // then Scheduler.execute() runs the idle command, which sets stop(); then writeActuators()
        // pushes... stop. The motor never moves and "Intake mode" reads IDLE. Every real intake
        // demo above goes through a command factory for exactly this reason.
        if (gamepad1.dpadRightWasPressed()) {
            run("direct call (nothing should happen)", instant(() -> robot.intake.intake()));
        }

        // ---- 10. cancelling ----
        // reset() clears the scheduler outright. Note it does NOT call end() on running commands,
        // so anything needing cleanup should be cancelled individually instead.
        if (gamepad1.dpadDownWasPressed()) {
            Scheduler.reset();
            robot.intake.defaultIdleCommand().schedule();
            lastAction = "cancelled all";
        }
    }

    private void run(String name, Command command) {
        lastAction = name;
        command.schedule();
    }
}
