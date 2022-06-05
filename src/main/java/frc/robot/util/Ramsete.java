package frc.robot.util;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;


public final class Ramsete{
    public static Command followPath(Drive drive, Trajectory trajectory){
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPose,
            new RamseteController(Constants.RamseteB, Constants.RamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSeconds, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.DRIVE_KINEMATICS, drive::getWheelSpeeds, new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
            new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel), drive::tankDriveVolts, drive
    
        );
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
    
}
