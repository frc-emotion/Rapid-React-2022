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
import frc.robot.subsystems.*;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static XboxController driverController;
  public static XboxController operatorController;
  public static RobotContainer container;
  Command teleopCommand;
  Command autoCommand;

  public static Trajectory twoBallOne;
  public static Trajectory threeBallOne;
  public static Trajectory forw;
  TrajectoryCreator creator;

  SendableChooser<Integer> m_chooser = new SendableChooser<>();
  CvSink cvSink;
  CvSource outputStream;

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
    twoBallOne = creator.generateTrajectory("2ball100.wpilib.json", "AutoPickTest#2");
    forw = creator.generateTrajectory("2ball2.wpilib.json", "GG");

    // All 3 Ball Paths
    threeBallOne = creator.generateTrajectory("3ball1.wpilib.json", "First 3 Ball Path");

    // All 4 Ball Paths
    Drive.m_field.getObject("Two Ball One").setTrajectory(twoBallOne);
    Drive.m_field.getObject("trajectorDos").setTrajectory(forw);

    m_chooser.setDefaultOption("(tested) Two Ball Auto", 1);
    m_chooser.addOption("One Ball Taxi", 2);
    m_chooser.addOption("Three Ball Auto", 3);

    SmartDashboard.putData(m_chooser);

    /** Creates Camera Server for Driver Cam */
    CameraServer.startAutomaticCapture();

    // cvSink = CameraServer.getVideo();
    // outputStream = CameraServer.putVideo("Blur", 640, 480);

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
        autoCommand = container.runOne();
        break;

      case 3:
        autoCommand = container.getThreeBall();
        break;

      default:
        autoCommand = container.runAuto();
        break;
    }



    
    // container.getAutonomousCommand();
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
