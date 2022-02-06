package frc.robot;

public class Constants {
   
    //USB ports 
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
}
