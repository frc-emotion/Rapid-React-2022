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

public class HoodSequence extends SequentialCommandGroup {

    boolean ready;

    public HoodSequence(Intake intake, Shooter shot, Indexer index, double maxTime) {

        Command shootTwo = 
            new StartEndCommand(() -> shot.setHoodAngle(Constants.SHOOTER_ANGLE_AUTO),() -> shot.stopHood(), shot).withTimeout(maxTime);
            addCommands(
                shootTwo);

    }

}
