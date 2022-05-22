package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.commands.Forward;
import frc.robot.commands.TurnToDegrees;
import frc.robot.misc.RunRamsete;
import frc.robot.subsystems.*;


/**
 * Two Ball Auto used at SDR and AVR
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

        Command auto = sequence(
                new InstantCommand(() -> intake.intakeDown()).withTimeout(0.5),
                parallel(
                        new StartEndCommand(() -> intake.autointakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(6),
                                sequence(
                                        deadline(
                                                RunRamsete.executeAuto(drive, traj),
                                                new HoodSequence(intake, shot, index, 4)
                                        ),
                                        new ShooterSequence(intake, shot, index, 2, 1),
                                        
                                        parallel(
                                        RunRamsete.executeAuto(drive, eject)
                                                //Intake until Bottom sensor triggered
                                        //        new StartEndCommand(() -> intake.intakeRollerReverse(), () -> intake.intakeRollerOff(), intake)
                                        //        .withTimeout(6)
                                        )
                                )
                )
        );
        addCommands(
                auto);

    }

}
