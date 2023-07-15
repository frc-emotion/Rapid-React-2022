// package frc.robot.subsystems.vision;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.util.LimeLight;
// import frc.robot.util.dashboard.TabManager;
// import frc.robot.util.dashboard.TabManager.SubsystemTab;

// import java.util.ArrayList;
// import java.util.List;

// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// /**
//  * Calculates Robot Pose from vision targets
//  * <br>
//  * </br>
//  * Use either: <br>
//  * </br>
//  * - DifferentialDrivePoseEstimator with UKF ~ 10ms for a 20ms loop cycle
//  * <br>
//  * </br>
//  * - Give Weights to wheel odometry/visual odometry; override when enough vision
//  * targets are visible
//  * <br>
//  * </br>
//  * <b> WIP
//  */
// public class Vision extends SubsystemBase {
//     private static final double circleFitPrecision = 0;
//     private static final int minTargetCount = 2;

//     private double[] cornerX = new double[] {};
//     private double[] cornerY = new double[] {};

//     private LimeLight limeLight;
//     private AHRS gyro;

//     public Translation2d finalTranslation2d;

//     public NetworkTableEntry calculatedPoseXData;
//     public NetworkTableEntry calculatedPoseYData;
//     public NetworkTableEntry currentAngle;
//     public NetworkTableEntry desiredAngleData;

//     public Vision(LimeLight limeLight, AHRS gyro) {
//         this.limeLight = limeLight;
//         initShuffleboard();
//     }

//     @Override
//     public void periodic() {
//         refreshCornerData();

//         int targetCount = limeLight.GetMode() == 3.0 ? cornerX.length / 4 : 0;

//         if (targetCount >= minTargetCount) {
//             List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
//             for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
//                 List<VisionCorner> corners = new ArrayList<>();
//                 double totalX = 0.0, totalY = 0.0;
//                 for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
//                     if (i < cornerX.length && i < cornerY.length) {
//                         corners.add(new VisionCorner(cornerX[i], cornerY[i]));
//                         totalX += cornerX[i];
//                         totalY += cornerY[i];
//                     }
//                 }
//                 VisionCorner targetAvg = new VisionCorner(totalX / 4, totalY / 4);
//                 corners = sortCorners(corners, targetAvg);

//                 for (int i = 0; i < corners.size(); i++) {
//                     Translation2d translation = cameraToTargetTranslation(
//                             corners.get(i), i < 2 ? VisionConstants.TARGET_HEIGHT_UPPER
//                                     : VisionConstants.TARGET_HEIGHT_LOWER);
//                     if (translation != null) {
//                         cameraToTargetTranslations.add(translation);
//                     }
//                 }
//             }

//             if (cameraToTargetTranslations.size() >= minTargetCount * 4) {
//                 finalTranslation2d = CircleFitter.fit(VisionConstants.TARGET_DIAMETER / 2.0,
//                         cameraToTargetTranslations, circleFitPrecision);
//             }
//         }

//         updateShuffleboard();
//     }

//     @Override
//     public void simulationPeriodic() {

//     }

//     public void refreshCornerData() {
//         List<Double> cornerXArrayList = new ArrayList<>();
//         List<Double> cornerYArrayList = new ArrayList<>();
//         if (limeLight.getTv() == 1.0) {
//             boolean returnXCorner = true;
//             for (double pixelCoordinate : limeLight.getCorners().getDoubleArray(new double[] {})) {
//                 if (returnXCorner) {
//                     cornerXArrayList.add(pixelCoordinate);
//                 } else {
//                     cornerYArrayList.add(pixelCoordinate);
//                 }
//                 returnXCorner = !returnXCorner;
//             }
//         }
//         synchronized (Vision.this) {
//             cornerX = cornerXArrayList.stream().mapToDouble(Double::doubleValue).toArray();
//             cornerY = cornerYArrayList.stream().mapToDouble(Double::doubleValue).toArray();
//         }
//     }

//     private List<VisionCorner> sortCorners(List<VisionCorner> corners, VisionCorner average) {
//         Integer topLeftIndex = null;
//         Integer topRightIndex = null;
//         double minPosRads = Math.PI;
//         double minNegRads = Math.PI;
//         for (int i = 0; i < corners.size(); i++) {
//             VisionCorner corner = corners.get(i);
//             double angleRad = new Rotation2d(corner.x - average.x, average.y - corner.y)
//                     .minus(Rotation2d.fromDegrees(90)).getRadians();
//             if (angleRad > 0) {
//                 if (angleRad < minPosRads) {
//                     minPosRads = angleRad;
//                     topLeftIndex = i;
//                 }
//             } else {
//                 if (Math.abs(angleRad) < minNegRads) {
//                     minNegRads = Math.abs(angleRad);
//                     topRightIndex = i;
//                 }
//             }
//         }

//         // Find lower corners
//         Integer lowerIndex1 = null;
//         Integer lowerIndex2 = null;
//         for (int i = 0; i < corners.size(); i++) {
//             boolean alreadySaved = false;
//             if (topLeftIndex != null) {
//                 if (topLeftIndex.equals(i)) {
//                     alreadySaved = true;
//                 }
//             }
//             if (topRightIndex != null) {
//                 if (topRightIndex.equals(i)) {
//                     alreadySaved = true;
//                 }
//             }
//             if (!alreadySaved) {
//                 if (lowerIndex1 == null) {
//                     lowerIndex1 = i;
//                 } else {
//                     lowerIndex2 = i;
//                 }
//             }
//         }

//         // Combine final list
//         List<VisionCorner> newCorners = new ArrayList<>();
//         if (topLeftIndex != null) {
//             newCorners.add(corners.get(topLeftIndex));
//         }
//         if (topRightIndex != null) {
//             newCorners.add(corners.get(topRightIndex));
//         }
//         if (lowerIndex1 != null) {
//             newCorners.add(corners.get(lowerIndex1));
//         }
//         if (lowerIndex2 != null) {
//             newCorners.add(corners.get(lowerIndex2));
//         }
//         return newCorners;
//     }

//     private Translation2d cameraToTargetTranslation(VisionCorner corner, double goalHeight) {
//         double halfWidthPixels = VisionConstants.LIMELIGHT_WIDTH_PIXELS / 2.0;
//         double halfHeightPixels = VisionConstants.LIMELIGHT_WIDTH_PIXELS / 2.0;
//         double nY = -((corner.x - halfWidthPixels) / halfWidthPixels);
//         double nZ = -((corner.y - halfHeightPixels) / halfHeightPixels);

//         Translation2d xzPlaneTranslation = new Translation2d(1.0, VisionConstants.LIMELIGHT_VPH / 2.0 * nZ)
//                 .rotateBy(VisionConstants.LIMELIGHT_ANGLE);
//         double x = xzPlaneTranslation.getX();
//         double y = VisionConstants.LIMELIGHT_VPW / 2.0 * nY;
//         double z = xzPlaneTranslation.getY();

//         double differentialHeight = VisionConstants.LIMELIGHT_MOUNTING_HEIGHT - goalHeight;
//         if ((z < 0.0) == (differentialHeight > 0.0)) {
//             double scaling = differentialHeight / -z;
//             double distance = Math.hypot(x, y) * scaling;
//             Rotation2d angle = new Rotation2d(x, y);
//             return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
//         }

//         return null;
//     }

//     /** Return calculated angle to turn to based on pose */
//     public double desiredRobotAngle() {

//         // double m1 = (getVisionPose().getY() -
//         // Constants.HUB_POSE.getY())/(getVisionPose().getX() -
//         // Constants.HUB_POSE.getX());
//         double m2 = 0;
//         double m3 = (getVisionPose().getY() - LLPose(1, gyro.getRotation2d().getRadians()).getY())
//                 / (getVisionPose().getX() - LLPose(1, gyro.getRotation2d().getRadians()).getX());

//         // Line desired = new Line(m1, Constants.HUB_POSE.getX(),
//         // Constants.HUB_POSE.getY());
//         Line horizontal = new Line(m2, Constants.HUB_POSE.getX(), Constants.HUB_POSE.getY());
//         Line current = new Line(m3, getVisionPose().getX(), getVisionPose().getY());

//         double a = distanceTo(new double[] { getVisionPose().getX(), getVisionPose().getY() },
//                 new double[] { Constants.HUB_POSE.getX(), Constants.HUB_POSE.getY() });
//         double b = distanceTo(new double[] { getVisionPose().getX(), getVisionPose().getY() },
//                 current.intersection(horizontal));
//         double c = distanceTo(new double[] { Constants.HUB_POSE.getX(), Constants.HUB_POSE.getY() },
//                 current.intersection(horizontal));

//         double angle = Units
//                 .radiansToDegrees(Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b)));

//         return angle;
//     }

//     /*
//      * public static double verifyDesiredRobotAngle(double[] hub, double[] p, double
//      * cAngle) {
//      * double[] ll = { tLLPose(p, 1, cAngle).getX(), tLLPose(p, 1, cAngle).getY() };
//      * 
//      * // double m1 = (p[1] - hub[1])/(p[0] - hub[0]);
//      * double m2 = 0;
//      * double m3 = (p[1] - ll[1]) / (p[0] - ll[0]);
//      * 
//      * // Line desired = new Line(m1, hub[0], hub[1]);
//      * Line horizontal = new Line(m2, hub[0], hub[1]);
//      * Line current = new Line(m3, p[0], p[1]);
//      * 
//      * double a = distanceTo(new double[] { p[0], p[1] }, new double[] { hub[0],
//      * hub[1] });
//      * double b = distanceTo(new double[] { p[0], p[1] },
//      * current.intersection(horizontal));
//      * double c = distanceTo(new double[] { hub[0], hub[1] },
//      * current.intersection(horizontal));
//      * 
//      * System.out.println(current.intersection(horizontal)[0] + ", " +
//      * current.intersection(horizontal)[1]);
//      * 
//      * // to rad
//      * double angle = Units
//      * .radiansToDegrees(Math.acos((Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c,
//      * 2)) / (2 * a * b)));
//      * 
//      * System.out.println(angle + ", " + a + ", " + b + ", " + c);
//      * 
//      * return angle;
//      * }
//      * 
//      * 
//      * // convert degrees to rad for correct angle
//      * public static Translation2d tLLPose(double[] c, double d, double ang) {
//      * return new Translation2d(c[0] + (d * Math.cos(ang)), c[1] + (d *
//      * Math.sin(ang)));
//      * }
//      */

//     private Translation2d LLPose(double d, double dAngle) {
//         return new Translation2d(getVisionPose().getX() + (d * Math.cos(Units.degreesToRadians(dAngle))),
//                 getVisionPose().getY() + (d * Math.sin(Units.degreesToRadians(dAngle))));
//     }

//     private static double distanceTo(double[] a, double[] b) {
//         return Math.sqrt(Math.pow(((b[0] - a[0])), 2) + Math.pow(((b[1] - a[1])), 2));
//     }

//     public Pose2d getVisionPose() {
//         return new Pose2d(finalTranslation2d, gyro.getRotation2d());
//     }

//     private void initShuffleboard(){
//         ShuffleboardTab visionTab = TabManager.getInstance().accessTab(SubsystemTab.VISION);

//         calculatedPoseXData = TabManager.getInstance().addWidget(visionTab, BuiltInWidgets.kTextView, "X Position", 0, 
//                 new int[] { 0, 0 }, new int[] { 2, 2 });

//         calculatedPoseYData = TabManager.getInstance().addWidget(visionTab, BuiltInWidgets.kTextView, "Y Position", 0,
//                 new int[] { 2, 0 }, new int[] { 2, 2 });
//         currentAngle = TabManager.getInstance().addWidget(visionTab, BuiltInWidgets.kTextView, "Robot Angle (GYRO)",
//                 0, new int[]{4, 0}, new int[]{2, 2});
//         desiredAngleData = TabManager.getInstance().addWidget(visionTab, BuiltInWidgets.kTextView, "desiredAngle Difference",
//                 0, new int[] { 6, 0 }, new int[] { 2, 2 });
//     }

//     private void updateShuffleboard() {
//         calculatedPoseXData.setDouble(getVisionPose().getX());
//         calculatedPoseYData.setDouble(getVisionPose().getY());
//         currentAngle.setDouble(gyro.getRotation2d().getDegrees());
//         desiredAngleData.setDouble(desiredRobotAngle());
//     }

//     public static class VisionCorner {
//         public final double x;
//         public final double y;

//         public VisionCorner(double x, double y) {
//             this.x = x;
//             this.y = y;
//         }
//     }

//     private static class Line {
//         public double m;
//         public double x, y;
//         public double b;

//         public Line(double m, double x, double y) {
//             this.m = m;
//             this.x = x;
//             this.y = y;
//             b = yIntercept();
//         }

//         public double yIntercept() {
//             return y - m * x;
//         }

//         public double solveY(double xValue) {
//             return (m * xValue) + b;
//         }

//         public double[] intersection(Line fx) {
//             double xPoint = (fx.b - b) / (m - fx.m);
//             double yPoint = fx.solveY(xPoint);
//             return new double[] { xPoint, yPoint };
//         }

//     }
// }
