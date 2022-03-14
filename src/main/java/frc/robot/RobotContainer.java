// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.ShooterTeleop;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final Drive drive = new Drive();
  private final Shooter shoot = new Shooter();
  private final Indexer index = new Indexer();
  private final Intake intaker = new Intake();


  DriveCommand drivetrain = new DriveCommand(drive);
  ShooterTeleop shooter = new ShooterTeleop(shoot);
  IndexTeleop indexer = new IndexTeleop(index);
  IntakeTeleop intake = new IntakeTeleop(intaker);

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

    Command runSubsystems = new ParallelCommandGroup(drivetrain, shooter, indexer, intake);
    return runSubsystems;
  }

  public Command runAuto(){
    autoPick = Robot.test;
    b2 = Robot.forw;

    return new TwoBallAuto(drive, intaker, shoot, index, autoPick, b2);
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