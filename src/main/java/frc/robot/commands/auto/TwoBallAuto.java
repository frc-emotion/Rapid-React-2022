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
public class TwoBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();

    boolean ready;
    BooleanSupplier x;

    double degrees = 45;

    public TwoBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = sequence(
                parallel(
                new InstantCommand(() -> intake.intakeDown()).withTimeout(2),
                new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(2.5)
                ),
                parallel(
                        new StartEndCommand(() -> intake.intakeRoller()
                        , () -> intake.intakeRollerOff(), intake)
                                .withTimeout(10),
                        sequence(
                                path.executeAuto(drive, traj),
                                parallel(
                                        new AutoShooter(shot, Constants.SHOOTER_RPM_CARGO_LINE, Constants.SHOOTER_ANGLE_AUTO, ready)
                                                .withTimeout(5),
                                        sequence( 
                                                new Forward(drive, -0.4).withTimeout(0.97),
                                                new TurnToDegrees(drive, 7.1, false).withTimeout(1),
                                                new InstantCommand(() -> index.indexForward(Constants.INDEXINGSPEED)).withTimeout(5)
                                        
                                        )      
                                ))));
        addCommands(
                auto);

    }

}
