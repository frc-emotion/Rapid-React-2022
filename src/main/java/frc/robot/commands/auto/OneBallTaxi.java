package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Forward;
import frc.robot.misc.RunRamsete;
import frc.robot.subsystems.*;


/**
 * One Ball Taxi (Shoot -> Drive Back)
 */
public class OneBallTaxi extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();

    boolean ready;

    public OneBallTaxi(Drive drive, Intake intake, Shooter shot, Indexer index) {
        //Reset encoder Pos
        drive.resetEncoders();

        //One Ball Sequence
        Command auto = sequence(
                new WaitCommand(1.5),
                new StartEndCommand(shot::autoZero, shot::stopHood, shot).withTimeout(3),
                parallel(
                        new AutoShooter(shot, Constants.SHOOTER_RPM_CARGO_LINE, Constants.SHOOTER_ANGLE_CARGO_LINE, ready)
                                                .withTimeout(5),
                            sequence(
                                new StartEndCommand(() -> index.indexForward(Constants.INDEXINGSPEED), () -> index.indexForward(Constants.INDEXINGSPEED), index).withTimeout(5),
                                new Forward(drive, 0.5).withTimeout(1)
                                )                                     
                                ));
        addCommands(
                auto);

    }

}
