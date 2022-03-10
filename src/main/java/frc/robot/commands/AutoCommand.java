package frc.robot.commands;

import java.lang.reflect.Executable;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RunRamsete;
import frc.robot.subsystems.Drive;

public class AutoCommand {
    RunRamsete path = new RunRamsete();

    public Command x(Drive drive, Trajectory Uno){
        return path.executeAuto(drive, Uno);
    }

    public SequentialCommandGroup runPath(Drive drive, Trajectory Uno, Trajectory Dos){
        return new SequentialCommandGroup(
                path.executeAuto(drive, Uno),
                path.executeAuto(drive, Dos)
            );
    }
    
    
}
