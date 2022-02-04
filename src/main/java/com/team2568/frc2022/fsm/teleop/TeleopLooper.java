package com.team2568.frc2022.fsm.teleop;

import com.team2568.frc2022.ILooper;

/**
 * Executes the runnables of all teleop FSMs and updates all output and state
 * registers
 */
public class TeleopLooper extends ILooper {
    private static TeleopLooper mLooper;

    public static TeleopLooper getInstance() {
        if (mLooper == null) {
            mLooper = new TeleopLooper();
        }
        return mLooper;
    }

    private TeleopLooper() {
        super("Teleop", 1);
        registerRunnables();
        registerStoppableRegisters();
    }
}
