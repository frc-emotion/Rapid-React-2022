package com.team2568.frc2022.registers;

import com.team2568.frc2022.commands.Set;

/**
 * A Register augments writing and reading of a singluar value.
 */

public class Register<T> {
	private T mValue;
	public Object mLock = new Object();

	private StringToValue<T> parser;

	public Register(StringToValue<T> parser) {
		this.parser = parser;
	}

	public void set(T value) {
		synchronized (mLock) {
			this.mValue = value;
		}
	}

	public T get() {
		synchronized (mLock) {
			return mValue;
		}
	}

	public Set<T> getSetter(String s) {
		return new Set<T>(this, parser.convert(s));
	}
}
