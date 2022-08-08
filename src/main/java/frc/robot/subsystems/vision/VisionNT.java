package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionNT {

    private double[] cornerX = new double[] {};
    private double[] cornerY = new double[] {};

    public final NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("ledMode");
    private final NetworkTableEntry pipelineEntry = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("pipeline");
    private final NetworkTableEntry validEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private final NetworkTableEntry dataEntry = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("tcornxy");

    public VisionNT() {
        List<Double> cornerXArrayList = new ArrayList<>();
        List<Double> cornerYArrayList = new ArrayList<>();

        if (validEntry.getDouble(0.0) == 1.0) {
            boolean returnXCorner = true;
            for (double pixelCoordinate : dataEntry.getDoubleArray(new double[] {})) {
                if (returnXCorner) {
                    cornerXArrayList.add(pixelCoordinate);
                } else {
                    cornerYArrayList.add(pixelCoordinate);
                }
                returnXCorner = !returnXCorner;
            }
        }
        synchronized (VisionNT.this) {
            cornerX = cornerXArrayList.stream().mapToDouble(Double::doubleValue).toArray();
            cornerY = cornerYArrayList.stream().mapToDouble(Double::doubleValue).toArray();

        }
    }

    public synchronized void updateCornerData(VisionData data) {
        data.cornerX = cornerX;
        data.cornerY = cornerY;
    }

    public void forceSetLeds(boolean enabled) {
        ledMode.forceSetDouble(enabled ? 3.0 : 1.0);
    }

    public void forceSetPipeline(int pipeline) {
        pipelineEntry.forceSetNumber(pipeline);
    }

    public class VisionData {
        public double[] cornerX = new double[] {};
        public double[] cornerY = new double[] {};
    }
}
