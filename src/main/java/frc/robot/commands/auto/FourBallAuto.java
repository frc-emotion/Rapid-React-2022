package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Forward;
import frc.robot.misc.RunRamsete;
import frc.robot.subsystems.*;

public class FourBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();
    boolean ready;

    public FourBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory traj2, Trajectory traj3, Trajectory traj4) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = 
        sequence(
                new InstantCommand(() -> intake.intakeDown()).withTimeout(0.5),
                new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(0.5),
                new WaitCommand(1),
                parallel(
                        new StartEndCommand(() -> intake.intakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(20),
                        sequence(
                                path.executeAuto(drive, traj),
                                parallel(
                                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready)
                                                .withTimeout(4),
                                        sequence(
                                               // new TurnToDegrees(drive, 20, true).withTimeout(2),
                                                new Forward(drive, -0.4).withTimeout(0.6)
                                        // new StartEndCommand(() -> intake.intakeR)
                                        ),
                                new InstantCommand(() -> index.indexForward(Constants.INDEXINGSPEED)).withTimeout(2)
                                ).withTimeout(2),

                                path.executeAuto(drive, traj2),
                                
                                path.executeAuto(drive, traj3),
                                parallel(
                                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready)
                                                .withTimeout(5),
                                        sequence(
                                        // new StartEndCommand(() -> intake.intakeR)
                                        ),
                                new InstantCommand(() -> index.indexForward(Constants.SHOOTINDEXINGSPEED))).withTimeout(2) 
                                )));

        addCommands(
                auto);

    }

    /**
     * 
     * 
     * parallel(
                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready).withTimeout(3),
                        new InstantCommand(() -> index.indexForward(Constants.INDEXINGSPEED)).withTimeout(3)
                ),
     */

}
