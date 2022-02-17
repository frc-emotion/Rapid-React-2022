package com.team2568.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Shooter extends Subsystem {
    private static Shooter mSubsystem = new Shooter();
    private TalonFX mL, mR;

    public static Shooter getSubsystem() {
        return mSubsystem;
    }

    private Shooter() {
    }

    /**
     * Read from results from registers and set motors etc.
     */
    public void setOutputs() {

    }

    /**
     * Write to status registers
     */
    public void writeStatus() {

    }

    /**
     * Output permanent statements
     */
    public void writeDashboard() {

    }

    /**
     * Debugging dashboard statements
     */
    public void outputTelemetry() {

    }
}
