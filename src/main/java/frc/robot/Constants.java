package frc.robot;

public class Constants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    public static final int[] DRIVE_RIGHT_PORTS = {1, 2 ,3};
    public static final int[] DRIVE_LEFT_PORTS = {4, 5, 6};

    public static final int NEO_MAX_LIMIT = 45;
    public static final double DRIVE_FORWARD_SPEED = 0.9;
    public static final double DRIVE_REGULAR_POWER = 0.75;
    public static final double DRIVE_SLOW_POWER = 0.4;
    public static final double DRIVE_TURBO_POWER = 0.9;


    //Climb Constants
    public static final int[] CLIMB_MOTORS_ID = {};
    public static final double CLIMB_THRESHOLD = 0.2;
    public static final double CLIMB_MOTOR_SPEED = 0.85;
    public static final double CLIMB_TARGET_POS = 0; //In revolutions
    public static final double CLIMB_ZERO_POS = 0;
    public static final double CLIMB_kP = 0;
    public static final double CLIMB_kI = 0;
    public static final double CLIMB_kD = 0;

    public static final int CLIMB_PNEUMATICS[]= {0, 1, 2, 3};
    
}
