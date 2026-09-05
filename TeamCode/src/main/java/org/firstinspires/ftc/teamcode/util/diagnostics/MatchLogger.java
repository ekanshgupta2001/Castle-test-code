package org.firstinspires.ftc.teamcode.util.diagnostics;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes one CSV row per loop to {@code /sdcard/FIRST/data/{tag}_{timestamp}.csv}.
 *
 * <p>Pull the file off the robot after a match and plot it. Most "the robot did something weird"
 * questions are answerable from the log and almost none are answerable from memory — telemetry
 * scrolls past at 50 Hz and nobody is watching it during a match anyway.
 *
 * <h2>Schema</h2>
 * {@code t_ms, phase, remaining_s, loop_ms, battery_v, pose_x, pose_y, pose_h, path_busy,
 * path_completion, trans_error, heading_error, intake_mode, intake_target_v, intake_actual_v,
 * intake_amps, unjamming, has_pollen, color_v, ll_target, ll_tx, ll_ty, localization, macro,
 * macro_outcome}
 *
 * <p>Three groups of columns earn their place for specific reasons. <b>Target and error</b>
 * ({@code path_completion}, {@code trans_error}, {@code heading_error}) are what separate "the robot
 * was here" from "the robot was tracking well" — pose alone cannot tell you whether the follower was
 * doing its job. <b>Identifiers</b> ({@code macro}, {@code macro_outcome}, {@code intake_mode})
 * record what the robot was <em>trying</em> to do, without which a log shows motion with no
 * intent behind it. And <b>{@code battery_v} with {@code loop_ms}</b> are the two numbers that
 * explain most "it behaved differently that time" reports.
 *
 * <h2>Two things it deliberately does</h2>
 * <ul>
 *   <li><b>Catches {@link Exception}, not just {@link IOException}.</b> A null pose or a hardware
 *       read fault would otherwise propagate out of {@code loop()} and end the match over a log
 *       line. Logging must never be able to stop the robot.</li>
 *   <li><b>Flushes every {@link #FLUSH_EVERY_ROWS} rows.</b> A {@code BufferedWriter} only reaches
 *       disk when its 8 KB buffer fills or it is closed, so an abnormal exit used to lose the tail
 *       of the match — the interesting part.</li>
 * </ul>
 *
 * <p>{@code Locale.US} is pinned when formatting: on a device set to a comma-decimal locale, the
 * numbers would otherwise be written with commas and silently corrupt the CSV.
 */
public class MatchLogger {
    private static final String DIR = "FIRST/data";
    private static final String[] HEADER = {
            "t_ms", "phase", "remaining_s", "loop_ms", "battery_v",
            "pose_x", "pose_y", "pose_h",
            "path_busy", "path_completion", "trans_error", "heading_error",
            "intake_mode", "intake_target_v", "intake_actual_v", "intake_amps", "unjamming",
            "has_pollen", "color_v",
            "ll_target", "ll_tx", "ll_ty", "localization",
            "macro", "macro_outcome"
    };

    /** Rows between disk flushes. Without this the tail of the match is lost on a crash. */
    private static final int FLUSH_EVERY_ROWS = 50;

    private final BufferedWriter writer;
    private final long startNanos;
    private final File file;
    private boolean closed = false;
    private int rowsSinceFlush = 0;

    public MatchLogger(String tag) throws IOException {
        File dir = new File(AppUtil.ROOT_FOLDER, DIR);
        if (!dir.exists() && !dir.mkdirs()) {
            throw new IOException("could not create " + dir);
        }
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        file = new File(dir, tag + "_" + stamp + ".csv");
        writer = new BufferedWriter(new FileWriter(file));
        writeRow((Object[]) HEADER);
        startNanos = System.nanoTime();
    }

    public File getFile() {
        return file;
    }

    /** Logs a row without loop timing, for callers that do not measure it. */
    public void logRow(Robot robot) {
        logRow(robot, Double.NaN);
    }

    /**
     * Logs one row.
     *
     * @param loopMs duration of the previous loop iteration, or {@code NaN} if not measured
     */
    public void logRow(Robot robot, double loopMs) {
        if (closed) return;
        Pose pose = robot.drivetrain.getPose();
        double x = pose == null ? Double.NaN : pose.getX();
        double y = pose == null ? Double.NaN : pose.getY();
        double h = pose == null ? Double.NaN : pose.getHeading();

        // Every follower-derived value is NaN rather than 0 when there is no follower, so a run on
        // a robot whose drivetrain failed to build is visibly missing data instead of looking like
        // a robot that sat perfectly still with zero error.
        Follower follower = robot.drivetrain.getFollower();
        double completion = Double.NaN;
        double transError = Double.NaN;
        double headingError = Double.NaN;
        if (follower != null) {
            completion = follower.getPathCompletion();
            Vector translational = follower.getTranslationalError();
            if (translational != null) transError = translational.getMagnitude();
            headingError = follower.getHeadingError();
        }

        MatchClock clock = robot.getMatchClock();
        String phase = clock == null ? "NONE" : clock.getPhase().toString();
        double remaining = clock == null ? Double.NaN : clock.getRemainingSeconds();

        try {
            writeRow(
                    (System.nanoTime() - startNanos) / 1_000_000L,
                    phase,
                    remaining,
                    loopMs,
                    robot.getBatteryVolts(),
                    x, y, h,
                    robot.drivetrain.isFollowingPath() ? 1 : 0,
                    completion,
                    transError,
                    headingError,
                    robot.intake.getMode(),
                    robot.intake.getTargetVelocity(),
                    robot.intake.getVelocityTicksPerSec(),
                    robot.intake.getCurrentAmps(),
                    robot.intake.isUnjamming() ? 1 : 0,
                    robot.intake.hasPollen() ? 1 : 0,
                    robot.colorSensor.getValue(),
                    robot.limelight.hasTarget() ? 1 : 0,
                    robot.limelight.getTx(),
                    robot.limelight.getTy(),
                    robot.poseFusion.getLastResult(),
                    robot.macros.getActiveName(),
                    robot.macros.getOutcome()
            );
            if (++rowsSinceFlush >= FLUSH_EVERY_ROWS) {
                writer.flush();
                rowsSinceFlush = 0;
            }
        } catch (Exception ignored) {
            // Deliberately catches Exception, not just IOException: a null pose or a hardware read
            // fault would otherwise propagate out of loop() and end the match over a log line.
        }
    }

    public void close() {
        if (closed) return;
        closed = true;
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
        }
    }

    private void writeRow(Object... cells) throws IOException {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < cells.length; i++) {
            if (i > 0) sb.append(',');
            sb.append(formatCell(cells[i]));
        }
        sb.append('\n');
        writer.write(sb.toString());
    }

    private static String formatCell(Object o) {
        if (o instanceof Double || o instanceof Float) {
            return String.format(Locale.US, "%.4f", ((Number) o).doubleValue());
        }
        // Enum and String cells (macro names, modes, localization results) are written straight
        // through; strip any comma so one stray character cannot shift every later column.
        return String.valueOf(o).replace(',', ';');
    }
}
