package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Ramsete;


/**
 * Two Ball Eject 
 * <b> WIP <b>
 * 
 */
public class TwoBallEject extends SequentialCommandGroup {
    boolean ready;
    BooleanSupplier x;

    double degrees = 45;

    public TwoBallEject(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory eject) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = 
                sequence(new InstantCommand(() -> intake.intakeDown()).withTimeout(0.5),
                parallel(new StartEndCommand(() -> intake.autointakeRoller(), () -> intake.intakeRollerOff(), intake) .withTimeout(6),
                sequence(
                        deadline(Ramsete.followPath(drive, traj),new HoodSequence(intake, shot, index, 4)), new ShooterSequence(intake, shot, index, 2, 1),
                                parallel(Ramsete.followPath(drive, eject))))
        );
        addCommands(
                auto);

    }

}
