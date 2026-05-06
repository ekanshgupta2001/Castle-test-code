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

public class MatchLogger {
    private static final String DIR = "FIRST/data";
    private static final String[] HEADER = {
            "t_ms", "pose_x", "pose_y", "pose_h",
            "intake_target_v", "intake_actual_v", "intake_amps", "has_pollen",
            "color_v", "ll_target", "ll_tx", "ll_ty"
    };

    private final BufferedWriter writer;
    private final long startNanos;
    private final File file;
    private boolean closed = false;

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
        try {
            writeRow(
                    (System.nanoTime() - startNanos) / 1_000_000L,
                    pose.getX(), pose.getY(), pose.getHeading(),
                    robot.intake.getTargetVelocity(),
                    robot.intake.getVelocityTicksPerSec(),
                    robot.intake.getCurrentAmps(),
                    robot.intake.hasPollen() ? 1 : 0,
                    robot.colorSensor.getValue(),
                    robot.limelight.hasTarget() ? 1 : 0,
                    robot.limelight.getTx(),
                    robot.limelight.getTy()
            );
        } catch (IOException ignored) {
            // Don't crash the match for a logging hiccup.
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
