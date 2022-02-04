package com.team2568.frc2022.registers;

/**
 * Actual value of the register will not be written until update is called.
 */
public class UpdateRegister<T> extends Register<T> {
	private T mCurrent;

	public UpdateRegister(StringToValue<T> parser) {
		super(parser);
	}

	@Override
	public void set(T value) {
		synchronized (mLock) {
			mCurrent = value;
		}
	}

	public void update() {
		synchronized (mLock) {
			super.set(mCurrent);
		}
	}
}
