// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.OneBallTaxi;
import frc.robot.commands.ShooterTeleop;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.ThreeBallAuto;
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

 // SendableChooser<Integer> m_chooser;
  Trajectory TwoBall;
  Trajectory OneBall;
  Trajectory ThreeBall;
  Trajectory b1;
  Trajectory b2;
  public RobotContainer() {

     //m_chooser.setDefaultOption("(tested) Two Ball Auto", 1);
    // m_chooser.addOption("One Ball Taxi", 2);

// Put the chooser on the dashboard
    //SmartDashboard.putData(m_chooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getTeleopCommand(){
    return new TeleopCommand(drive, index, intaker, shoot, climb);
  }

  public Command runOne(){
    return new OneBallTaxi(drive, intaker, shoot, index);
  }

  public Command runAuto(){
    TwoBall = Robot.twoBallOne;

    //if (m_chooser.getSelected() == 1){
    return new TwoBallAuto(drive, intaker, shoot, index, TwoBall);
    //}

    //else {
    //  return new OneBallTaxi(drive, intaker, shoot, index);
    //}
    
    //b2 = Robot.forw;
    
  }

  public Command getThreeBall(){
    ThreeBall = Robot.threeBallOne;

    return new ThreeBallAuto(drive, intaker, shoot, index, ThreeBall, b2);
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