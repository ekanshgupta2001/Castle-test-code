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

/**
 * Teaching OpMode: how the Ivy command system works, one button at a time.
 *
 * <p>Run it, press each button, and watch the telemetry. Every binding below demonstrates exactly
 * one idea, in the order they are worth learning. Read the code alongside the robot.
 *
 * <p>Read {@code docs/03-commands.md} first if any of this is unfamiliar.
 */
@TeleOp(name = "Concept: Commands", group = "Concept")
public class ConceptCommands extends OpMode {
    private Robot robot;
    private String lastAction = "none";

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
    }

    @Override
    public void loop() {
        robot.readSensors();      // 1. observe
        handleButtons();          // 2. decide
        Scheduler.execute();      //    run every scheduled command exactly once
        robot.writeActuators();   // 3. act

        telemetry.addLine("A = instant      B = wait 1s then run");
        telemetry.addLine("X = sequential   Y = parallel");
        telemetry.addLine("LB = race        RB = deadline");
        telemetry.addLine("DPAD-UP = conditional   LB = conditional skip   DPAD-DOWN = cancel all");
        telemetry.addLine();
        telemetry.addData("Last scheduled", lastAction);
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
        if (gamepad1.aWasPressed()) {
            run("instant", instant(() -> robot.intake.eject()));
        }

        // ---- 2. sequential with a wait: steps run strictly in order ----
        if (gamepad1.bWasPressed()) {
            run("waitMs then intake", sequential(
                    waitMs(1000),
                    instant(() -> robot.intake.intake())
            ));
        }

        // ---- 3. sequential: each step waits for the previous one to finish ----
        if (gamepad1.xWasPressed()) {
            run("sequential", sequential(
                    robot.intake.runForMs(1500, 400),
                    waitMs(200),
                    robot.intake.runForMs(-1400, 400),
                    instant(() -> robot.intake.stop())
            ));
        }

        // ---- 4. parallel: everything runs at once, group ends when ALL are done ----
        // Note both children here touch different subsystems. Two commands requiring the SAME
        // subsystem cannot run in parallel — Ivy will resolve the conflict, not run both.
        if (gamepad1.yWasPressed()) {
            run("parallel", parallel(
                    robot.intake.runForMs(1500, 800),
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

        // ---- 7. conditional behaviour, chosen when the command RUNS ----
        // Building the branch at schedule time would capture the wrong answer; putting the check
        // inside an instant defers it to execution.
        if (gamepad1.dpadUpWasPressed()) {
            run("conditional", instant(() -> {
                if (robot.intake.hasPollen()) robot.intake.eject();
                else robot.intake.intake();
            }));
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
        // first tick. MainAuto.skipIfAnyLegMissed is the real use of this.
        if (gamepad1.leftBumperWasPressed()) {
            run("conditional skip", conditional(
                    () -> robot.intake.hasPollen(),
                    instant(() -> lastAction = "skipped: already carrying"),
                    robot.intake.intakeCommand()));
        }

        // ---- 9. cancelling ----
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
