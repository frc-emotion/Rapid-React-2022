package com.team2568.frc2022.registers;

/**
 * Set can only be called once. Additonal calls of set will not change the
 * value.
 */
public class SetOnceRegister<T> extends Register<T> {
    private boolean isSet = false;

    public SetOnceRegister(StringToValue<T> parser) {
        super(parser);
    }

    @Override
    public void set(T value) {
        if (!isSet) {
            super.set(value);
            isSet = true;
        }
    }
}
