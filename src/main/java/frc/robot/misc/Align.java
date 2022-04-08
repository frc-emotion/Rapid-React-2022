package frc.robot.misc;
import frc.robot.Constants;

public class Align {
    private LimeLight limeLight;

    public Align() {
        limeLight = new LimeLight();
    }

    public void disable(){
        limeLight.disableLight();
    }

    public void enable(){
        limeLight.enableLight();
    }

    public double getLed(){
        return limeLight.GetMode();
    }

    public boolean targetFound() {
        limeLight.selectPipeline(Constants.PORT_PIPELINE);

        if (limeLight.getTv() == 0) {
            return false;
        }
        return true;
    }

    public double getError() {
        if (targetFound()) {
            return -limeLight.getTx();
        }
        return -1;
    }
}