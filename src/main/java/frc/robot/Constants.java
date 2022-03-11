package frc.robot;

import frc.robot.limelight.Distance;

public class Constants {

    // controllers
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double TRIGGER_THRESHOLD = 0.3;
    public static final double JOYSTICK_THRESHOLD = 0.02;

    // current limits
    public static final int NEO_MAX_CURRENT = 45;
    public static final int TALON_MAX_CURRENT = 40;

    // intake
    public static final int INTAKE_PORT = 7;
    public static final int[] PNEUMATIC_INTAKE_PORTS = { 12, 3, 8, 7 };
    public static final double INTAKE_SPEED = 0.4;

    // indexer
    public static final int INDEXERFALCON = 13;
    public static final int BOTTOMSENSOR = 0;
    public static final int TOPSENSOR = 1;
    public static final double INDEXINGSPEED = 0.4;

    // drivetrain
    public static final int[] DRIVE_RIGHT_PORTS = { 1, 2, 3 };
    public static final int[] DRIVE_LEFT_PORTS = { 4, 5, 6 };

    public static final double DRIVE_FORWARD_SPEED = 0.9;
    public static final double DRIVE_REGULAR_POWER = 0.65;
    public static final double DRIVE_SLOW_POWER = 0.4;
    public static final double DRIVE_TURBO_POWER = 0.9;

    // Climb Constants
    public static final int[] CLIMB_MOTORS_ID = { 11, 12 };
    public static final double CLIMB_MOTOR_SPEED = 0.85;
    public static final double CLIMB_ZERO_POS = 0;
    public static final int CLIMB_POWER_CHANNEL = 0;
    public static final double CLIMB_DRAW_THRESHOLD = 0;

    public static final double CLIMB_TARGET_MAX_POS = 46 * 2048; // rev
    public static final double CLIMB_TARGET_MIN_POS = 5 * 2048; // rev
    public static final double CLIMB_MAX_POS = 47; // rev
    public static final double CLIMB_MIN_POS = 3; // rev

    public static final double CLIMB_kP = 0;
    public static final double CLIMB_kI = 0;
    public static final double CLIMB_kD = 0;

    public static final int CLIMB_PNEUMATICS[] = { 5, 10, 9, 6};

    // Shooter Constants
    public static final int SHOOTER_HOOD_PORT = 8;
    public static final int SHOOTER_LEFT_PORT = 9;
    public static final int SHOOTER_RIGHT_PORT = 10;

    public static final int SHOOTER_LIMIT_PORT = 2;

    // NEED TO EXPERIMETNALLY GET
    public static final double SHOOTER_RPM_UP_AGAINST = 1500;
    public static final double SHOOTER_RPM_CARGO_LINE = 2000;
    public static final double SHOOTER_RPM_CLOSE_PAD = 2500;
    public static final double SHOOTER_RPM_FAR_PAD = 3000;

    // NEED TO EXPERIMETNALLY GET
    public static final double SHOOTER_ANGLE_UP_AGAINST = 10;
    public static final double SHOOTER_ANGLE_CARGO_LINE = 15;
    public static final double SHOOTER_ANGLE_CLOSE_PAD = 20;
    public static final double SHOOTER_ANGLE_FAR_PAD = 0;

    public static final double SHOOTER_THRESHOLD_RPM = 200;

    public static final double SHOOTER_NOMINAL_VOLTAGE = 7;
    public static final double SHOOTER_HOOD_NOMINAL_VOLTAGE = 7;

    public static final double SHOOTER_KS = 0.74904;
    public static final double SHOOTER_KV = 0.12235;
    public static final double SHOOTER_KA = 0.062135;

    public static final double SHOOTER_KP = 0.003793;
    public static final double SHOOTER_KD = 0;

    // NEED TO GET WITH SYS ID
    public static final double SHOOTER_HOOD_KP = 0.65;
    public static final double SHOOTER_HOOD_KD = 0;

    public static final double SHOOTER_HOOD_MIN = 2.5;
    public static final double SHOOTER_HOOD_MAX = 40;

    public static final double SHOOTER_HOOD_SPEED = 0.3;
    public static final double SHOOTER_HOOD_ZERO_SPEED = 0.15;

    public static final double SHOOTER_REV_TO_ANGLE = 0.416;
    public static final double SHOOTER_STARTING_ANGLE = 0;

    // Physical Constants
    public static final double SHOOTER_HEIGHT = 0.813; // meter
    public static final double SHOOTER_TARGET_THRESHOLD = 0.1; // meter

    // REFERENCE ANGLES
    public static final double MOUNTING_HEIGHT = 0.381; // tochange
    public static final double MOUNTING_ANGLE = 0.5759; // tochange
    public static final double REFERENCE_HEIGHT = 2.64;

    public static final Distance DISTANCE = new Distance(MOUNTING_HEIGHT, MOUNTING_ANGLE, REFERENCE_HEIGHT);

    public static final int PORT_PIPELINE = 1;
}
