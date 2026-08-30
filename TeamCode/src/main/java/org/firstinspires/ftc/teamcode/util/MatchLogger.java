package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;

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
 * {@code t_ms, pose_x, pose_y, pose_h, intake_target_v, intake_actual_v, intake_amps, has_pollen,
 * color_v, ll_target, ll_tx, ll_ty}
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
            "t_ms", "pose_x", "pose_y", "pose_h",
            "intake_target_v", "intake_actual_v", "intake_amps", "has_pollen",
            "color_v", "ll_target", "ll_tx", "ll_ty"
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

    public void logRow(Robot robot) {
        if (closed) return;
        Pose pose = robot.drivetrain.getPose();
        double x = pose == null ? Double.NaN : pose.getX();
        double y = pose == null ? Double.NaN : pose.getY();
        double h = pose == null ? Double.NaN : pose.getHeading();
        try {
            writeRow(
                    (System.nanoTime() - startNanos) / 1_000_000L,
                    x, y, h,
                    robot.intake.getTargetVelocity(),
                    robot.intake.getVelocityTicksPerSec(),
                    robot.intake.getCurrentAmps(),
                    robot.intake.hasPollen() ? 1 : 0,
                    robot.colorSensor.getValue(),
                    robot.limelight.hasTarget() ? 1 : 0,
                    robot.limelight.getTx(),
                    robot.limelight.getTy()
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
        return String.valueOf(o);
    }
}
