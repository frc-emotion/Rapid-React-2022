package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Calculates Robot Pose from vision targets
 * <br></br>
 * Use either: <br></br>
 * - DifferentialDrivePoseEstimator with UKF ~ 10ms for a 20ms loop cycle
 * <br></br>
 * - Give Weights to wheel odometry/visual odometry; override when enough vision targets are visible
 * <br></br> <b> WIP
 */
public class Vision extends SubsystemBase {

    public Vision(){}

    /**Uses Limelight Smart-Grouping feature*/
    public void calculatePosefromCorners(){}

    /**Return calculated angle to turn to based on pose*/
    public void returnDesiredRobotPose(){}
}
