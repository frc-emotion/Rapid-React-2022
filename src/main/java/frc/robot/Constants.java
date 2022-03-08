package frc.robot;

public class Constants {

    // controllers
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double TRIGGER_THRESHOLD = 0.3;
    public static final double JOYSTICK_THRESHOLD = 0.2;

    // current limits
    public static final int NEO_MAX_CURRENT = 45;
    public static final int TALON_MAX_CURRENT = 40;

    // intake
    public static final int INTAKE_PORT = 7;
    public static final int[] PNEUMATIC_INTAKE_PORTS = { 0, 0, 0, 0 }; // CHANGE
    public static final double INTAKE_SPEED = 0.4;

    // drivetrain
    public static final int[] DRIVE_RIGHT_PORTS = { 1, 2, 3 };
    public static final int[] DRIVE_LEFT_PORTS = { 4, 5, 6 };

    public static final double DRIVE_FORWARD_SPEED = 0.9;
    public static final double DRIVE_REGULAR_POWER = 0.65;
    public static final double DRIVE_SLOW_POWER = 0.4;
    public static final double DRIVE_TURBO_POWER = 0.9;

    // Climb Constants
    public static final int[] CLIMB_MOTORS_ID = { 11, 12 };
    public static final double CLIMB_THRESHOLD = 0.2;
    public static final double CLIMB_MOTOR_SPEED = 0.85;
    public static final double CLIMB_ZERO_POS = 0;

    public static final double CLIMB_TARGET_MAX_POS = 46 * 2048; // rev
    public static final double CLIMB_TARGET_MIN_POS = 5 * 2048; // rev
    public static final double CLIMB_MAX_POS = 47; // rev
    public static final double CLIMB_MIN_POS = 3; // rev

    public static final double CLIMB_kP = 0;
    public static final double CLIMB_kI = 0;
    public static final double CLIMB_kD = 0;

    public static final int CLIMB_PNEUMATICS[] = { 10, 5, 2, 13 };

    // Shooter Constants
    public static final int SHOOTER_INDEXER_PORT = 13;
    public static final int SHOOTER_HOOD_PORT = 8;
    public static final int SHOOTER_LEFT_PORT = 9;
    public static final int SHOOTER_RIGHT_PORT = 10;

    public static final int SHOOTER_BOTTOM_SENSOR_PORT = 0;
    public static final int SHOOTER_TOP_SENSOR_PORT = 1;

    public static final int SHOOTER_LIMIT_PORT = 2;

    // NEED TO EXPERIMETNALLY GET
    public static final double SHOOTER_RPM_UP_AGAINST = 3000;
    public static final double SHOOTER_RPM_CARGO_LINE = 3000;
    public static final double SHOOTER_RPM_CLOSE_PAD = 3000;
    public static final double SHOOTER_RPM_FAR_PAD = 3000;

    // NEED TO EXPERIMETNALLY GET
    public static final double SHOOTER_ANGLE_UP_AGAINST = 0;
    public static final double SHOOTER_ANGLE_CARGO_LINE = 0;
    public static final double SHOOTER_ANGLE_CLOSE_PAD = 0;
    public static final double SHOOTER_ANGLE_FAR_PAD = 0;

    public static final double SHOOTER_THRESHOLD_RPM = 200;

    public static final double SHOOTER_NOMINAL_VOLTAGE = 7;

    public static final double SHOOTER_KS = 0.70133;
    public static final double SHOOTER_KV = 0.11419;
    public static final double SHOOTER_KA = 0.013578;

    public static final double SHOOTER_KP = 0.012615;
    public static final double SHOOTER_KD = 0;

    // NEED TO GET WITH SYS ID
    public static final double SHOOTER_HOOD_KP = 0;
    public static final double SHOOTER_HOOD_KI = 0;
    public static final double SHOOTER_HOOD_KD = 0;

    public static final double SHOOTER_HOOD_MIN = 5;
    public static final double SHOOTER_HOOD_MAX = 35;

    public static final double SHOOTER_HOOD_SPEED = 0.3;
    public static final double SHOOTER_INDEX_SPEED = 0.4;

    public static final double SHOOTER_REV_TO_ANGLE = 0.416;

}
