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


    public void updatePoseByCamTran(){}

    /** Get raw corner data */
    public void seperateCorners(){
        

        ledOn = vision.ledMode.getDouble(1.0) == 3.0 ? true : false;
        int targetCount = ledOn ? data.cornerX.length / 4 : 0;

    }


    public void cameraToTargetTranslation(){


    }

    /** Return calculated angle to turn to based on pose */
    public void returnDesiredRobotPose() {
    }


    //Uses circle fitter - references 6328 code
    public void estimateHubLocation(){

    }



    public static class VisionCorners {
        public final double x;
        public final double y;

        public VisionCorners(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
}
