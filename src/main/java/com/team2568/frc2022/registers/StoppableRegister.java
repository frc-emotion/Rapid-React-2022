package com.team2568.frc2022.registers;

/**
 * Sets the current value to the passed stop value when requested.
 */
public class StoppableRegister<T> extends ResetableRegister<T> {
    private T mStopValue;

    public StoppableRegister(StringToValue<T> parser, T resetValue, T stopValue) {
        super(parser, resetValue);
        this.mStopValue = stopValue;
    }

    public void stop() {
        set(mStopValue);
    }

    public T getStop() {
        return mStopValue;
    }
}
