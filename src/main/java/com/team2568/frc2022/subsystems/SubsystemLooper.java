package com.team2568.frc2022.subsystems;

import com.team2568.frc2022.ILooper;

/**
 * Looper which executes the subsystem runnables and updates status registers
 */
public class SubsystemLooper extends ILooper {
    private static SubsystemLooper mLooper;

    public static SubsystemLooper getInstance() {
        if (mLooper == null) {
            mLooper = new SubsystemLooper();
        }
        return mLooper;
    }

    private SubsystemLooper() {
        super("Subsystem");
        registerRunnables();

        registerUpdateRegisters();
    }
}
