package com.team2568.frc2022.registers;

/**
 * Sets the current value to the passed reset value when requested.
 */
public class ResetableRegister<T> extends UpdateRegister<T> {
    private T mResetValue;

    public ResetableRegister(StringToValue<T> parser, T resetValue) {
        super(parser);
        this.mResetValue = resetValue;
    }

    public void reset() {
        set(mResetValue);
    }

    public T getReset() {
        return mResetValue;
    }
}
