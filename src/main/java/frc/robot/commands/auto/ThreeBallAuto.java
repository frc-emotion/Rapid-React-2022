package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.misc.TurnToDegrees;
import frc.robot.commands.teleop.Forward;
import frc.robot.subsystems.*;
import frc.robot.util.Ramsete;

/**
 * Unorganzied CommandGroup for Three Ball Auto Route
 * <br></br>
 * <b>NO ACCURATE TRAJECTORIES PLOTTED (DO NOT RUN) </b>
 *
 */
public class ThreeBallAuto extends SequentialCommandGroup {
    boolean ready;

    public ThreeBallAuto(Drive drive, Intake intake, Shooter shot, Indexer index, Trajectory traj, Trajectory traj2) {
        // Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = sequence(
                new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(0.5),
                parallel(
                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready).withTimeout(3),
                        new InstantCommand(() -> index.indexForward(Constants.INDEXINGSPEED)).withTimeout(3)
                ),
                new WaitCommand(1),
                new InstantCommand(() -> intake.intakeDown()).withTimeout(1),
                parallel(
                        new StartEndCommand(() -> intake.intakeRoller(), () -> intake.intakeRollerOff(), intake)
                                .withTimeout(10),
                        sequence(
                                Ramsete.followPath(drive, traj),
                                parallel(
                                        new AutoShooter(shot, 2200, Constants.SHOOTER_ANGLE_CARGO_LINE, ready)
                                                .withTimeout(5),
                                        sequence(
                                                new TurnToDegrees(drive, 20, true).withTimeout(2),
                                                new Forward(drive, -0.4).withTimeout(0.6)
                                        ),
                                new InstantCommand(() -> index.indexForward(Constants.INDEXINGSPEED))).withTimeout(3) 
                                )));

        addCommands(
                auto);

    }

}
