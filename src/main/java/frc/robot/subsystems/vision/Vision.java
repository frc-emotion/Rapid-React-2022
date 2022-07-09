package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimeLight;
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
    public LimeLight limeLight;

    public Vision(LimeLight limeLight){
        this.limeLight = limeLight;
    }
    /**Uses Limelight Smart-Grouping feature to get raw corner data */
    public void seperateCorners(){}

    public void cameraToTarget(){}

    public void calculateCurrentPose(){}

    /**Return calculated angle to turn to based on pose*/
    public void returnDesiredRobotPose(){}

    /**Turn to desired pose */
    public void turnToPose(){}
}
