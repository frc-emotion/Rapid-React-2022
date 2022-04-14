package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/**
 * Abstracted CommandGroup for revving up Shooter + Indexing Balls during Auto
 */
public class ShooterSequence extends SequentialCommandGroup {

    boolean ready;

    public ShooterSequence(Intake intake, Shooter shot, Indexer index, double maxTime, double lastIndexTime) {

        Command shootTwo = parallel(
            new StartEndCommand(shot::autoShootTime, shot::stop, shot).withTimeout(maxTime),
            sequence(
                new StartEndCommand(() -> index.indexForward(Constants.SHOOTINDEXINGSPEED), () -> index.indexStop(), index).withTimeout(1),
                new WaitCommand(0.1), 
                new StartEndCommand(() -> index.indexForward(Constants.SHOOTINDEXINGSPEED), () -> index.indexStop(), index).withTimeout(1)
            )

        );
            addCommands(
                shootTwo);

    }

}
