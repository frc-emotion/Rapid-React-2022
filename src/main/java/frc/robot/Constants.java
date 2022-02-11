package frc.robot;

public class Constants {
   
    //USB ports 
    public static final int DRIVER_PORT = 0;
    public static int OPERATOR_PORT = 1;

    //controller constants
    public static double TRIGGER_THRESHOLD = 0.3; //from IRC

    //CANbus ports 
    public static int INTAKE_PORT = 0; //CHANGE AFTER ELECTRICAL  

    //PH ports 
    public static int[] PNEUMATIC_INTAKE_PORTS = {0,0,0,0}; //CHANGE ELECTRICAL

    //current limits 
    public static int INTAKE_LIMIT = 35; //from IRC
    
    //intake constants 
    public static double INTAKE_SPEED = 0.8; //from IRC 
    

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

    //Indexer Constants
    public static final double PULLEYDIAMETER = 1.504; //inches
    public static final double PULLEYCIRCUMFRENCE = 4.724955351; //inches
    public static final int INDEXERFALCON = 7;
    public static final int BOTTOMSENSOR = 0;
    public static final int TOPSENSOR = 1;
    public static final double INDEXINGSPEED = 0.5;
    
}
