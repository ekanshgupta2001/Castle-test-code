package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.firstinspires.ftc.teamcode.util.diagnostics.Drawing;
import org.firstinspires.ftc.teamcode.util.diagnostics.LoopTimer;
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.diagnostics.RateLimiter;

import java.io.IOException;
import java.util.List;

/**
 * Everything a match OpMode has to do, done once: building the robot, resetting the scheduler,
 * opening a log, initialising the dashboard, resetting gamepad edge detection, starting the match
 * clock, timing the loop, rate-limiting the drawing, and closing it all down again. Two copies of
 * a lifecycle would be two places for it to drift (see {@code docs/09}).
 *
 * <h2>The loop order is enforced here, not remembered</h2>
 * {@link #loop()} is {@code final}. Subclasses fill in the gaps between the fixed steps:
 *
 * <pre>
 *   readSensors()        1. observe   &lt;- fixed
 *   updateLocalization()
 *   onDecide()                        &lt;- yours: read buttons, schedule commands
 *   Scheduler.execute()  2. decide    &lt;- fixed
 *   writeActuators()     3. act       &lt;- fixed
 *   onAfterAct()                      &lt;- yours: haptics, pose handoff
 *   log / telemetry / draw            &lt;- fixed, draw is rate-limited
 * </pre>
 *
 * This is the single most important rule in the codebase (see {@code docs/01}). A subclass cannot
 * get it wrong — there is no ordering left for it to choose.
 *
 * <p>One {@link #nowMs} timestamp is taken per loop and shared, rather than each caller reading the
 * clock again.
 */
@Configurable
public abstract class MatchOpMode extends OpMode {
    /**
     * How often the Panels field view is redrawn, in milliseconds.
     *
     * <p>Each redraw builds and sends a network packet. Nobody can see 50 frames a second, and the
     * time comes straight out of the control loop.
     */
    public static long DRAW_INTERVAL_MS = 100;

    /**
     * How often telemetry is transmitted to the Driver Station, in milliseconds.
     *
     * <p>The SDK default is 250 ms. Set explicitly so the rate is a decision rather than an
     * inherited default, and so it can be lowered while debugging.
     */
    public static int TELEMETRY_INTERVAL_MS = 100;

    protected Robot robot;
    protected MatchLogger logger;
    /** Non-null when the log file could not be opened; surfaced by subclasses in telemetry. */
    protected String loggerError = null;

    /** Duration of the previous loop, in milliseconds. */
    protected double loopMs = 0;
    /** Loop-time statistics for the whole run: p95, max, spike count. */
    protected final LoopTimer loopStats = new LoopTimer();
    /** One timestamp per loop, from the robot's {@code Clock}, so nothing re-reads it mid-cycle. */
    protected long nowMs = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final RateLimiter drawLimiter = new RateLimiter(DRAW_INTERVAL_MS);

    // ---- Subclass contract ----

    /** Filename prefix for this OpMode's match log, e.g. {@code "teleop"}. */
    protected abstract String logTag();

    /** Which match period this OpMode runs in, for the match clock. */
    protected abstract MatchClock.Period matchPeriod();

    /** Built subsystems are available; schedule default commands here. */
    protected void onInit() {}

    /** Runs after {@code readSensors()} on every init loop. Menus, localisation, warnings. */
    protected void onInitLoop() {}

    /** Runs once on START, after edge detection is reset and the clock has started. */
    protected void onStart() {}

    /** Read inputs and schedule commands. Runs before the scheduler, on fresh sensor data. */
    protected void onDecide() {}

    /** Runs after actuators are written. Haptics, pose handoff — anything that observes the result. */
    protected void onAfterAct() {}

    /** Add this OpMode's telemetry. Called every loop. */
    protected void onTelemetry() {}

    /** Draw to the Panels field view. Rate-limited to {@link #DRAW_INTERVAL_MS}. */
    protected void onDraw() {}

    /** Runs first in {@code stop()}, while the robot is still live. */
    protected void onStop() {}

    // ---- Fixed lifecycle ----

    @Override
    public final void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);

        try {
            logger = new MatchLogger(logTag());
        } catch (IOException e) {
            // A missing SD card must not cost a match. Record it and carry on unlogged.
            logger = null;
            loggerError = e.getMessage();
        }

        Drawing.init();
        onInit();

        reportMissingHardware();
        telemetry.update();
    }

    @Override
    public final void init_loop() {
        robot.readSensors();
        onInitLoop();
    }

    @Override
    public final void start() {
        // *WasPressed() latches until read, and init_loop() reads none of them. Without this, every
        // button bumped during init fires at once on the first loop tick - including the ones that
        // launch a macro.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.startMatch(matchPeriod());
        loopTimer.reset();
        onStart();
    }

    @Override
    public final void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopStats.record(loopMs);
        nowMs = robot.getClock().nowMs();

        robot.readSensors();          // 1. observe
        robot.updateLocalization();   //    blend any AprilTag fix into the pose estimate
        onDecide();                   // 2. decide
        Scheduler.execute();          //    driver control and macros both run here
        robot.writeActuators();       // 3. act

        onAfterAct();
        if (logger != null) logger.logRow(robot, loopMs);
        onTelemetry();

        // Rate-limited: the control loop above must not pay full price for a dashboard frame.
        drawLimiter.setIntervalMs(DRAW_INTERVAL_MS);
        if (drawLimiter.ready(nowMs)) onDraw();
    }

    @Override
    public final void stop() {
        onStop();
        Scheduler.reset();
        if (robot != null) robot.stop();
        if (logger != null) logger.close();
    }

    /** Lists any configuration name that could not be resolved. Empty output means all present. */
    protected void reportMissingHardware() {
        List<String> missing = robot.getMissingHardware();
        if (missing.isEmpty()) {
            telemetry.addLine("All hardware present.");
            return;
        }
        telemetry.addLine("MISSING HARDWARE (the robot will still run):");
        for (String name : missing) telemetry.addLine("  - " + name);
    }
}
