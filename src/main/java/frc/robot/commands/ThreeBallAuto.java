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
import frc.robot.RunRamsete;
import frc.robot.subsystems.*;

public class ThreeBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();
    boolean ready;

    public ThreeBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory traj2) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = sequence(
                new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(4),
                new WaitCommand(1),
                parallel(
                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready).withTimeout(5),
                        new InstantCommand(() -> index.indexForward()).withTimeout(3)
                ),
                new InstantCommand(() -> intake.intakeDown()).withTimeout(2),
                parallel(
                        new StartEndCommand(() -> intake.intakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(10),
                        sequence(
                                path.executeAuto(drive, traj),
                                parallel(
                                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready)
                                                .withTimeout(5),
                                        sequence(
                                                new TurnToDegrees(drive, 20, true)).withTimeout(2),
                                                new Forward(drive, -0.4).withTimeout(0.6)
                                        // new StartEndCommand(() -> intake.intakeR)
                                        ),

                                //new StartEndCommand(() -> index.indexForward(), () -> index.indexerStop(),
                                new InstantCommand(() -> index.indexForward()).withTimeout(3))));

        // Command runTwoBall = parallel(trajectories); //add intake and then have
        // intake/indexer run always
        addCommands(
                auto);

    }

}
