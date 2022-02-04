package com.team2568.frc2022.subsystems;

import com.team2568.frc2022.Registers;

/**
 * Subsystems should have one instance and be registered to the SubsystemManager
 * instance. Outputs (Motors) should not be set until the setOutput call to
 * avoid glitches during computation.
 * 
 * @author Ryan Chaiyakul
 */
public abstract class Subsystem {
    private final Runnable mRunnable = new Runnable() {
        @Override
        public void run() {
            setOutputs();
            writeStatus();
            writeDashboard();

            if (Registers.kTelemetry.get()) {
                outputTelemetry();
            }
        }

    };

    /**
     * Get private Runnable variable from this subsystem
     */
    public Runnable getRunnable() {
        return mRunnable;
    }

    /**
     * Helper function to limit speed inputs from [-1, 1]
     */
    public double applyLimit(double speed) {
        if (speed > 1) {
            return 1;
        } else if (speed < -1) {
            return -1;
        }
        return speed;
    }

    /**
     * Read from results from registers and set motors etc.
     */
    public abstract void setOutputs();

    /**
     * Write to status registers
     */
    public abstract void writeStatus();

    /**
     * Output permanent statements
     */
    public abstract void writeDashboard();

    /**
     * Debugging dashboard statements
     */
    public abstract void outputTelemetry();

}
