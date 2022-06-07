// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.*;

/**
 *
 */
public class Robot extends TimedRobot {

  public static XboxController driverController;
  public static XboxController operatorController;
  public static RobotContainer container;
  private Command teleopCommand;
  private Command autoCommand;

  public static Trajectory twoBallOne;
  public static Trajectory twoBallEject;
  public static Trajectory threeBallOne;
  public static Trajectory forw;

  public static Trajectory fourBallOne;
  public static Trajectory fourBallTwo;
  public static Trajectory fourBallThree;
  private TrajectoryCreator creator;

  private SendableChooser<Integer> m_chooser = new SendableChooser<>();

  public InterpolatingTreeMap test;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driverController = new XboxController(Constants.DRIVER_PORT);
    operatorController = new XboxController(Constants.OPERATOR_PORT);

    container = new RobotContainer();
    creator = new TrajectoryCreator();

    // All Two Ball Paths
    twoBallOne = creator.generateTrajectory("2ballSmooth.wpilib.json", "Smooth 2 Ball Auto"); // TODO: Switch to
                                                                                              // Pathplanner

    twoBallEject = creator.generateTrajectory("2BallEject.wpilib.json", " TwoBall Eject Red"); // Build seperate path
                                                                                               // with lower accel
    // All 3 Ball Paths
    threeBallOne = creator.generateTrajectory("3ball1.wpilib.json", "First 3 Ball Path");

    // All 4 Ball Paths
    fourBallOne = creator.generateTrajectory("4Ball1.wpilib.json", "First 4 Ball Path");
    fourBallTwo = creator.generateTrajectory("4Ball2.wpilib.json", "2nd 4 Ball Path");
    fourBallThree = creator.generateTrajectory("4Ball3.wpilib.json", "Third 4 Ball Path");

    m_chooser.setDefaultOption("Two Ball Auto", 1);
    m_chooser.addOption("Two Ball Eject", 2);
    m_chooser.addOption("One Ball Taxi", 3);
    m_chooser.addOption("(UNTESTED) Three Ball Auto", 4);
    m_chooser.addOption("Four Ball Auto", 5);

    SmartDashboard.putData("Auto Path?", m_chooser);

    // Creates Camera Server for Driver Cam
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    switch (m_chooser.getSelected()) {
      case 1:
        autoCommand = container.runAuto();
        break;
      case 2:
        autoCommand = container.getTwoBallEject();
        break;
      case 3:
        autoCommand = container.runOne();
        break;
      case 4:
        autoCommand = container.getThreeBall();
        break;
      case 5:
        autoCommand = container.getFourBall();
        break;
      default:
        autoCommand = container.runAuto();
        break;
    }

    if (autoCommand != null) {
      autoCommand.schedule();
    }

    if (teleopCommand != null) {
      teleopCommand.cancel();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    teleopCommand = container.getTeleopCommand();

    if (autoCommand != null) {
      autoCommand.cancel();
    }
    if (teleopCommand != null) {
      teleopCommand.schedule();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
