package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

/**
 * Sequential command (Set Hood Angle with Timeout time)
 */
public class HoodSequence extends SequentialCommandGroup {

    boolean ready;

    public HoodSequence(Intake intake, Shooter shot, Indexer index, double maxTime) {

        Command shootTwo = 
            new StartEndCommand(() -> shot.setHoodAngle(Constants.SHOOTER_ANGLE_AUTO),() -> shot.stopHood(), shot).withTimeout(maxTime);
            addCommands(
                shootTwo);

    }

}
