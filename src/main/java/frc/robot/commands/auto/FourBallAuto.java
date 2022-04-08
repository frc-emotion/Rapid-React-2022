package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

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

    BooleanSupplier top = () -> false;

    boolean x;

    public FourBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory traj2, Trajectory traj3, Trajectory traj4) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = 
        sequence(

                new InstantCommand(() -> intake.intakeDown()).withTimeout(0.3),
               //new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(0.5),
                parallel(
                        new StartEndCommand(() -> intake.autointakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(13),
                        
                                sequence(
                                deadline(
                                        path.executeAuto(drive, traj),
                                        new HoodSequence(intake, shot, index, 4)
                                ),
                                new ShooterSequence(intake, shot, index, 2),

                                path.executeAuto(drive, traj2),

                                new StartEndCommand(() -> index.autoIndex(x), () -> index.indexStop(), index).withTimeout(0.8),      
                                
                                path.executeAuto(drive, traj3),

                                new ShooterSequence(intake, shot, index, 4))));
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
