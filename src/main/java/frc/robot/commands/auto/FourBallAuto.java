package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Ramsete;

public class FourBallAuto extends SequentialCommandGroup {
    boolean ready;

    BooleanSupplier top = () -> false;

    boolean x;

    public FourBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory traj2, Trajectory traj3, Trajectory traj4) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        //drive.resetEncoders();

        //Creates a command with both parallel and sequential commands to achieve a 4 ball command
        Command auto = 
        sequence(
                new InstantCommand(() -> intake.intakeDown()).withTimeout(0.5),
                parallel(
                        new StartEndCommand(() -> intake.autointakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(16),
                                sequence(
                                        deadline(
                                                Ramsete.followPath(drive, traj),
                                                new HoodSequence(intake, shot, index, 4)
                                        ),
                                new ShooterSequence(intake, shot, index, 2, 1),
                                Ramsete.followPath(drive, traj2),
                                new StartEndCommand(() -> index.autoIndex(x), () -> index.indexStop(), index).withTimeout(0.8),       
                                Ramsete.followPath(drive, traj3),
                                new ShooterSequence(intake, shot, index, 4, 4))));
        addCommands(auto);

    }
}
