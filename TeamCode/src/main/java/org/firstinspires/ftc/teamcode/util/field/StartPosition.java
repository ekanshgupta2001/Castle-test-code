package org.firstinspires.ftc.teamcode.util.field;

import org.firstinspires.ftc.teamcode.util.AutoSelector;

/**
 * Where on the alliance wall the robot starts.
 *
 * <p><b>Placeholder.</b> Rename these to the real BIOBUZZ tile names once the field layout is known
 * — a value called {@code LEFT} tells a driver nothing at 9 am on competition day.
 *
 * <p>Selected during init by {@link AutoSelector}; maps to a starting pose via
 * {@link FieldConstants#startPose}. Adding a third position needs a matching preset there and
 * nothing else — {@code AutoSelector} cycles whatever values exist.
 */
public enum StartPosition {
    LEFT,
    RIGHT
}
