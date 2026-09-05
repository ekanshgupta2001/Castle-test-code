package org.firstinspires.ftc.teamcode.util.hardware;

/**
 * Every name this code expects to find in the Robot Controller configuration.
 *
 * <h2>Why these are not just literals in each constructor</h2>
 * They used to be — {@code "IM"} in Intake, {@code "limelight"} in Limelight, {@code "sensor_color"}
 * in ColorSensor, and the drive motors inside Pedro's constants. Four files to open to answer "what
 * does the config need to be called?", and four files to edit when someone renames a device in the
 * configuration app at a competition.
 *
 * <p>Renaming a device is now a one-line change here. The subsystem constructors that take an
 * explicit name still exist, so a second robot with a different configuration is still possible
 * without touching this file.
 *
 * <h2>Keep these matching the physical robot</h2>
 * A name here that does not exist in the active configuration does not crash anything — every
 * lookup goes through {@link Hardware}, which records it and lets the subsystem no-op. Run the
 * {@code SelfTest} OpMode and it will print exactly which of these could not be found.
 */
public final class HardwareNames {
    private HardwareNames() {}

    // ---- Subsystem devices ----

    /** goBILDA 5203 435 RPM intake motor. */
    public static final String INTAKE_MOTOR = "IM";
    /** Limelight 3A, connected over USB-C. */
    public static final String LIMELIGHT = "limelight";
    /** REV/AndyMark colour sensor watching the intake. */
    public static final String COLOR_SENSOR = "sensor_color";

    /** Encoder motor driving the lift. Absent on the current robot; the subsystem no-ops. */
    public static final String LIFT_MOTOR = "lift";
    /** Servo driving the claw. Absent on the current robot; the subsystem no-ops. */
    public static final String CLAW_SERVO = "claw";

    // ---- Drivetrain ----
    //
    // These are consumed by MecanumConstants inside pedro/Constants.java, which is still
    // default-constructed. When that file is filled in, point it at these rather than retyping the
    // names — otherwise the drivetrain becomes the one subsystem whose config names live somewhere
    // else again.

    public static final String FRONT_LEFT_MOTOR = "front_left_drive";
    public static final String FRONT_RIGHT_MOTOR = "front_right_drive";
    public static final String BACK_LEFT_MOTOR = "back_left_drive";
    public static final String BACK_RIGHT_MOTOR = "back_right_drive";

    /** goBILDA Pinpoint odometry computer, if one is used as the localizer. */
    public static final String PINPOINT = "pinpoint";
    /** Control Hub IMU, for localizers that use it directly. */
    public static final String IMU = "imu";
}
