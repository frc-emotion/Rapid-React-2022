package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class AutoConstants {
    public static final double DRIVE_KD = 0.05;
    public static final double DRIVE_KP = 0.5;
    public static final double DRIVE_KI = 0;

    public static final double ksVolts = 0.19735;
    public static final double kvVoltSeconds = 2.513;// 2.4936;
    public static final double kaVoltSecondsSquaredPerMeter = 0.38003;

    // Tuned kP valeue for 2022
    public static final double kPDriveVel = 0;// 3.3077;
    public static final double kDDriveVel = 0;

    // Kinematics
    public static final double TrackWidthMeters = 0.6928;// TORPEDO: 0.5369051588705906;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TrackWidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double maxSpeedMPS = 0.4;
    public static final double maxAccelerationMPSsq = 1;

    // Ramsete Parameters

    public static final double RamseteB = 2.0;
    public static final double RamseteZeta = 0.7;
    public static final double kDriveGearRatio = 9.642;// 10.3846;
  }

  // GENERAL CONSTANTS

  // controllers
  public static final int DRIVER_PORT = 0;
  public static final int OPERATOR_PORT = 1;
  public static final double TRIGGER_THRESHOLD = 0.3;
  public static final double JOYSTICK_THRESHOLD = 0.2;

  // current limits
  public static final int NEO_MAX_CURRENT = 45;
  public static final int TALON_MAX_CURRENT = 40;

  // intake
  public static final int INTAKE_PORT = 10;
  public static final int[] PNEUMATIC_INTAKE_PORTS = { 7, 3 }; // 0, 6 };
  public static final double INTAKE_SPEED = 0.50;
  public static final double REVERSE_INTAKE_SPEED = 0.65;

  // indexer
  public static final int INDEXERFALCON = 20;
  public static final int BOTTOMSENSOR = 0;
  public static final int TOPSENSOR = 1;
  public static final int INTAKESENSORR = 3;
  public static final int INTAKESENSORL = 4;
  public static final double INDEXINGSPEED = 0.4;
  public static final double SHOOTINDEXINGSPEED = 0.6;

  // drivetrain
  public static final int[] DRIVE_RIGHT_PORTS = { 4, 5, 6 };
  public static final int[] DRIVE_LEFT_PORTS = { 1, 9, 3 };

  public static final double DRIVE_FORWARD_SPEED = 0.65;
  public static final double DRIVE_REGULAR_POWER = 0.65;
  public static final double DRIVE_SLOW_POWER = 0.4;
  public static final double DRIVE_TURBO_POWER = 0.9;

  // Climb Constants
  public static final int[] CLIMB_MOTORS_ID = { 11, 12 };
  public static final double CLIMB_MOTOR_SPEED = 0.85;
  public static final double CLIMB_ZERO_POS = 0;
  public static final int CLIMB_POWER_CHANNEL = 14;
  public static final double CLIMB_DRAW_THRESHOLD = 0;

  public static final double CLIMB_TARGET_MAX_POS = 46 * 2048; // rev
  public static final double CLIMB_TARGET_MIN_POS = 5 * 2048; // rev
  public static final double CLIMB_MAX_POS = 125; // rev
  public static final double CLIMB_MIN_POS = 0; // rev

  public static final double CLIMB_kP = 0;

  public static final double CLIMB_kI = 0;
  public static final double CLIMB_kD = 0;

  public static final int CLIMB_PNEUMATICS[] = { 5, 1, 9, 6 };

  // Shooter Constants
  public static final int SHOOTER_HOOD_PORT = 8;
  public static final int SHOOTER_PORT = 12;
  // public static final int SHOOTER_RIGHT_PORT = 10;

  public static final int SHOOTER_LIMIT_PORT = 2;

  // NEED TO EXPERIMETNALLY GET

  // NEED TO EXPERIMETNALLY GET
  public static final double SHOOTER_RPM_FENDER_LOW = 800;
  public static final double SHOOTER_ANGLE_FENDER_LOW = 38;

  public static final double SHOOTER_RPM_FENDER_HIGH = 1600;
  public static final double SHOOTER_ANGLE_FENDER_HIGH = 1;

  public static final double SHOOTER_RPM_CARGO_LINE = 1600;
  public static final double SHOOTER_ANGLE_CARGO_LINE = 11.5;

  public static final double SHOOTER_RPM_AUTO = 1750;
  public static final double SHOOTER_ANGLE_AUTO = 18;

  // Test Macros
  public static final double SHOOTER_RPM_OUTSIDE_TARMAC = 1750;
  public static final double SHOOTER_ANGLE_OUTSIDE_TARMAC = 18;

  public static final double SHOOTER_RPM_CLOSE_PAD = 1820;
  public static final double SHOOTER_ANGLE_CLOSE_PAD = 0;

  public static final double SHOOTER_RPM_FAR_PAD = 3000;
  public static final double SHOOTER_ANGLE_FAR_PAD = 0;

  public static final double SHOOTER_THRESHOLD_RPM = 200;

  public static final double SHOOTER_NOMINAL_VOLTAGE = 7;
  public static final double SHOOTER_HOOD_NOMINAL_VOLTAGE = 7;

  public static final double SHOOTER_KS = 0.74904;
  public static final double SHOOTER_KV = 0.12235;
  public static final double SHOOTER_KA = 0.062135;

  public static final double SHOOTER_KP = 0.7;
  public static final double SHOOTER_KD = 0;

  // FOR STEALTH WHEELS
  /*
   * public static final double SHOOTER_RPM_FENDER_LOW = 1100;
   * public static final double SHOOTER_ANGLE_FENDER_LOW = 40;
   * 
   * public static final double SHOOTER_RPM_FENDER_HIGH = 1850;
   * public static final double SHOOTER_ANGLE_FENDER_HIGH = 2.466;
   * 
   * public static final double SHOOTER_RPM_CARGO_LINE = 1900;
   * public static final double SHOOTER_ANGLE_CARGO_LINE = 5;
   * 
   * public static final double SHOOTER_RPM_OUTSIDE_TARMAC = 2022;
   * public static final double SHOOTER_ANGLE_OUTSIDE_TARMAC = 11;
   * 
   * 
   * public static final double SHOOTER_RPM_CLOSE_PAD = 2100;
   * public static final double SHOOTER_ANGLE_CLOSE_PAD = 0;
   * 
   * public static final double SHOOTER_RPM_FAR_PAD = 3000;
   * public static final double SHOOTER_ANGLE_FAR_PAD = 0;
   * 
   * public static final double SHOOTER_THRESHOLD_RPM = 200;
   * 
   * public static final double SHOOTER_NOMINAL_VOLTAGE = 7;
   * public static final double SHOOTER_HOOD_NOMINAL_VOLTAGE = 7;
   * 
   * public static final double SHOOTER_KS = 0.69337;
   * public static final double SHOOTER_KV = 0.12293;
   * public static final double SHOOTER_KA = 0.0041655;
   * 
   * public static final double SHOOTER_KP = 0.00056201;
   * public static final double SHOOTER_KD = 0;
   */


  // NEED TO GET WITH SYS ID
  public static final double SHOOTER_HOOD_KP = 0.65;
  public static final double SHOOTER_HOOD_KD = 0;

  public static final double SHOOTER_HOOD_MIN = 0.8;
  public static final double SHOOTER_HOOD_MAX = 40;

  public static final double SHOOTER_HOOD_SPEED = 0.3;
  public static final double SHOOTER_HOOD_ZERO_SPEED = 0.2;

  // Calculated from Solidworks (Manual Plot Test) and Motion Simulation
  public static final double SHOOTER_REV_TO_ANGLE = 0.416;
  public static final double SHOOTER_STARTING_ANGLE = 0;

  // Physical Constants
  public static final double SHOOTER_HEIGHT = 0.813; // meter
  public static final double SHOOTER_TARGET_THRESHOLD = 0.1; // meter

  // REFERENCE ANGLES
  public static final double MOUNTING_HEIGHT = 0.381; // tochange
  public static final double MOUNTING_ANGLE = 0.5759; // tochange
  public static final double REFERENCE_HEIGHT = 2.64;
 
  public static final Translation2d HUB_POSE = new Translation2d(8.2296, 4.1148);


  // public static final Distance DISTANCE = new Distance(MOUNTING_HEIGHT,
  // MOUNTING_ANGLE, REFERENCE_HEIGHT);
  public static final int PORT_PIPELINE = 1;

  public static class VisionConstants {
    public static final Rotation2d LIMELIGHT_ANGLE = Rotation2d.fromDegrees(0); // to change
    public static final double LIMELIGHT_MOUNTING_HEIGHT = Units.inchesToMeters(0);
    public static final double LIMELIGHT_VPW = 2.0 * Math.tan(Math.toRadians(59.6 / 2.0));
    public static final double LIMELIGHT_VPH = 2.0 * Math.tan(Math.toRadians(49.7 / 2.0));
    public static final int LIMELIGHT_WIDTH_PIXELS = 960;
    public static final int LIMELIGHT_HEIGHT_PIXELS = 720;
    public static final double TARGET_HEIGHT_LOWER = Units.inchesToMeters(8.0 * 12 + 5.625);
    public static final double TARGET_HEIGHT_UPPER = TARGET_HEIGHT_LOWER + Units.inchesToMeters(2.0);
    public static final double TARGET_DIAMETER = Units.inchesToMeters(4.0 * 12.0 + 5.375);
  }

}
