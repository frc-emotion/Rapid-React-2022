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
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.TwoBallAuto;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final Drive drive = new Drive();
  private final Shooter shoot = new Shooter();
  private final Indexer index = new Indexer();
  private final Intake intaker = new Intake();
  private final Climb climb = new Climb();


  Trajectory TwoBall;
  Trajectory b1;
  Trajectory b2;
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getTeleopCommand(){
    return new TeleopCommand(drive, index, intaker, shoot, climb);
  }

  public Command runAuto(){
    TwoBall = Robot.twoBallOne;
    //b2 = Robot.forw;
    return new TwoBallAuto(drive, intaker, shoot, index, TwoBall);
  }

  public Command getAutonomousCommand() {
    TwoBall = Robot.twoBallOne;
    //b2 = Robot.forw;
    return new TwoBallAuto(drive, intaker, shoot, index, TwoBall);

  }

  /*
  public Command getSubsytemChecker(){

    return new SubsystemChecker
  }
  */

}