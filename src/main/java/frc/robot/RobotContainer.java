// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.ejml.equation.Sequence;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Drive;

public class RobotContainer {

  private RunRamsete auto;

  private final Drive drive = new Drive();
  DriveCommand drivetrain = new DriveCommand(drive);

  Trajectory autoPick;
  Trajectory b1;
  Trajectory b2;
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getTeleopCommand(){
    return drivetrain;
  }

  public Command runAuto(){
    autoPick = Robot.test;
    b2 = Robot.forw;

    return new TwoBallAuto(drive, autoPick, b2);
  }





  public Command getAutonomousCommand() {

     /**
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSeconds, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.DRIVE_KINEMATICS, 10);

    TrajectoryConfig config = new TrajectoryConfig(Constants.maxSpeedMPS, Constants.maxAccelerationMPSsq)
        .setKinematics(Constants.DRIVE_KINEMATICS).addConstraint(autoVoltageConstraint);

  
     * TrajectoryGenerator.generateTrajectory( // new Pose2d(0,0, new
     * Rotation2d(0)), // List.of( new Translation2d(1,1), new Translation2d(2,-1)
     * ), new Pose2d(3,0, new Rotation2d(0)), config
     * 
     * );
    */
    autoPick = Robot.test;


    RamseteCommand ramseteCommand = new RamseteCommand(autoPick, drive::getPose,
        new RamseteController(Constants.RamseteB, Constants.RamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSeconds, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.DRIVE_KINEMATICS, drive::getWheelSpeeds, new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel), drive::tankDriveVolts, drive

    );

    drive.resetOdometry(autoPick.getInitialPose());
    
    return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));

    

  }

}