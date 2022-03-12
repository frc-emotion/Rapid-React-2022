package frc.robot.commands;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RunRamsete;
import frc.robot.subsystems.*;



public class TwoBallAuto extends SequentialCommandGroup {
    RunRamsete path = new RunRamsete();

    boolean ready;

    public TwoBallAuto(Drive drive, Intake intake, Shooter shot, Trajectory traj, Trajectory traj2){
        //Reset Position
        drive.resetOdometry(traj.getInitialPose());
        drive.resetEncoders();

        Command auto = 
        sequence(
            new InstantCommand(() -> intake.intakeDown()),
        parallel(
            new StartEndCommand(() -> intake.intakeRoller(), () -> intake.intakeRollerOff(), intake)
            .withTimeout(10),
           //ADD INDEXER
                //new InstantCommand(() ->drive.zeroHeading()),
            sequence(
                path.executeAuto(drive, traj),

                parallel(
                    new AutoShooter(shot, Constants.SHOOTER_RPM_CARGO_LINE, Constants.SHOOTER_ANGLE_CARGO_LINE, ready).withTimeout(5)
                    ,path.executeAuto(drive, traj2))
                    )
            )
        );

       // Command runTwoBall = parallel(trajectories); //add intake and then have intake/indexer run always
        addCommands(
           auto
        );
        
    }
    
}
