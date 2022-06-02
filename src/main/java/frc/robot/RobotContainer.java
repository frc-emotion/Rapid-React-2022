// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.auto.FourBallAuto;
import frc.robot.commands.auto.OneBallTaxi;
import frc.robot.commands.auto.ThreeBallAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.auto.TwoBallEject;
import frc.robot.commands.teleop.TeleopCommand;
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


  //Define a command to match trajectories to auton routines
  Trajectory TwoBall;
  Trajectory TwoBallEject;
  Trajectory OneBall;
  Trajectory ThreeBall;

  Trajectory b1;
  Trajectory b2;

  Trajectory four1;
  Trajectory four2;  
  Trajectory four3;
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getTeleopCommand() {
    return new TeleopCommand(drive, index, intaker, shoot, climb);
  }

  public Command runOne() {
    return new OneBallTaxi(drive, intaker, shoot, index);
  }

  public Command runAuto() {
    TwoBall = Robot.twoBallOne;
    return new TwoBallAuto(drive, intaker, shoot, index, TwoBall);
  }

  public Command getTwoBallEject(){
    TwoBall = Robot.twoBallOne;
    TwoBallEject = Robot.twoBallEject;
    return new TwoBallEject(drive, intaker, shoot, index, TwoBall, TwoBallEject);
  }

  public Command getThreeBall() {
    ThreeBall = Robot.threeBallOne;

    return new ThreeBallAuto(drive, intaker, shoot, index, ThreeBall, b2);
  }

  public Command getAutonomousCommand() {
    TwoBall = Robot.twoBallOne;
    return new TwoBallAuto(drive, intaker, shoot, index, TwoBall);

  }

  public Command getFourBall() {
    four1 = Robot.fourBallOne;
    four2 = Robot.fourBallTwo;
    four3 = Robot.fourBallThree;

    return new FourBallAuto(drive, intaker, shoot, index, four1, four2, four3, b2);
  }
}