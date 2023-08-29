package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TeleopCommand extends ParallelCommandGroup{

    public TeleopCommand(Drive drive, Indexer index, Intake intake, Shooter shoot){


        addCommands(
            new DriveCommand(drive),
            new ShooterTeleop(shoot),
            new IntakeTeleop(intake),
            new IndexTeleop(index)
        );
    }
    
}
