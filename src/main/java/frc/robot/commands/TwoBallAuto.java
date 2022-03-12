package frc.robot.commands;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RunRamsete;
import frc.robot.subsystems.*;


public class TwoBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();

    public TwoBallAuto(Drive drive, Trajectory traj, Trajectory traj2){
        //Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command trajectories = sequence(path.executeAuto(drive, traj),
        //new InstantCommand(() -> drive.zeroHeading()),
        path.executeAuto(drive, traj2));

        addCommands(
           trajectories
        );
        
    }
    
}
