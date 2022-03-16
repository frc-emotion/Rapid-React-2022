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

public class TwoBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();

    boolean ready;
    boolean x;

    double degrees = 45;

    public TwoBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = sequence(
                new WaitCommand(2.3),
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
                                                new Forward(drive, -0.4).withTimeout(0.8),
                                                new TurnToDegrees(drive, 16.8, false)
                                        // new WaitCommand(2.4),
                                        // new StartEndCommand(() -> index.indexForward(), () -> index.indexerStop(),
                                        // index).withTimeout(10)
                                        ),
                                        // new StartEndCommand(() -> index.indexForward(), () -> index.indexerStop(),
                                        // index).withTimeout(10)

                                        new InstantCommand(() -> index.indexForward()).withTimeout(10)

                                ))));

        // Command runTwoBall = parallel(trajectories); //add intake and then have
        // intake/indexer run always
        addCommands(
                auto);

    }

}
