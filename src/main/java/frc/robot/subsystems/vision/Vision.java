package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionNT.VisionData;
import frc.robot.util.Distance;
import frc.robot.util.LimeLight;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Calculates Robot Pose from vision targets
 * <br>
 * </br>
 * Use either: <br>
 * </br>
 * - DifferentialDrivePoseEstimator with UKF ~ 10ms for a 20ms loop cycle
 * <br>
 * </br>
 * - Give Weights to wheel odometry/visual odometry; override when enough vision
 * targets are visible
 * <br>
 * </br>
 */
public class Vision extends SubsystemBase {

    //Can remove
    private static final Rotation2d horizontalPlaneToLens = Rotation2d.fromDegrees(VisionConstants.LIMELIGHT_ANGLE);
    private static final double limeHeight = VisionConstants.LIMELIGHT_MOUNTING_HEIGHT;

    static final int minTargetCount = 2;

    public VisionNT vision;
    public VisionData data;

    private double vpw = 2.0 * Math.tan(Math.toRadians(59.6/2.0));
    private double vph = 2.0 * Math.tan(Math.toRadians(49.7/2.0));

    private boolean robotAimed;
    private boolean ledOn;

    public LimeLight limeLight;
    public Distance distance;
    private AHRS gyro;

    public Vision(LimeLight limeLight, Distance distance, AHRS gyro) {
        // Already instantiated
        this.limeLight = limeLight;
        this.distance = distance;
        this.gyro = gyro;
    }

    @Override
    public void periodic() {
        vision.updateCornerData(data);
        
    }

    @Override
    public void simulationPeriodic() {
        //jank code for glass 
    }

    public void updatePose() {
    }

    public void updatePoseByCamTran(){}

    /** Get raw corner data */
    public void seperateCorners(){

        ledOn = VisionNT.ledMode.getDouble(1.0) == 3.0 ? true : false;
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
