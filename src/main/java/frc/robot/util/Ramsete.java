package frc.robot.util;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;


public final class Ramsete{
//Uses pose estimator
    public static Command followPath(Drive drive, Trajectory trajectory){
        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drive::getPoseEstimate,
            new RamseteController(AutoConstants.RamseteB, AutoConstants.RamseteZeta),
            new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSeconds, AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.DRIVE_KINEMATICS, drive::getWheelSpeeds, new PIDController(AutoConstants.kPDriveVel, 0, AutoConstants.kDDriveVel),
            new PIDController(AutoConstants.kPDriveVel, 0, AutoConstants.kDDriveVel), drive::tankDriveVolts, drive
    
        );
        return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
    
}
