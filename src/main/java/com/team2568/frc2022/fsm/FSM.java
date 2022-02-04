package com.team2568.frc2022.fsm;

import com.team2568.frc2022.Registers;

/**
 * Abstract class for FSMs to be run by a looper
 */
public abstract class FSM {
    private final Runnable mRunnable = new Runnable() {
        @Override
        public void run() {
            compute();

            if (Registers.kTelemetry.get()) {
                writeDashboard();
            }
        }
    };

    /**
     * Get the private Runnable of this FSM
     * 
     * @return Runnable
     */
    public Runnable getRunnable() {
        return mRunnable;
    }

    /**
     * Executed every cycle
     */
    public abstract void compute();

    /**
     * Executed every cycle when telemetery(simulation) is true
     */
    public abstract void writeDashboard();
}
