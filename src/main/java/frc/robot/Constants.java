package frc.robot;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {

    public static final double ksVolts = 0.27717;
    public static final double kvVoltSeconds = 4.2592;
    public static final double kaVoltSecondsSquaredPerMeter= 0.6315;

    //Tuned kP valeue for 2022RR
    public static final double kPDriveVel = 0;//1.856 Assume sim does not count for enough error, as STD of sim vaules have not been calculated
    public static final double kDDriveVel = 0.1;
   /* 
    public static final double ksVolts = 0.164;
    public static final double kvVoltSeconds = 2.71;
    public static final double kaVoltSecondsSquaredPerMeter= 0.335;

    //Tuned kP valeue for Torpedo
    public static final double kPDriveVel = 2.3;
*/

    //Kinematics
    public static final double TrackWidthMeters = 0.6928;//0.5369051588705906;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TrackWidthMeters);

    //Max Trajectory Velocity/Acceleration
    public static final double maxSpeedMPS = 0.4;
    public static final double maxAccelerationMPSsq = 1;

    //Ramsete Parameters

    public static final double RamseteB = 2.0;
    public static final double RamseteZeta = 0.7;

    //public static final double driveConversionFactor = 0;

    public static final double kDriveGearRatio = 10.3846;

    
}
