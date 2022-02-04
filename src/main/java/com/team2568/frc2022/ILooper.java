package com.team2568.frc2022;

import java.util.ArrayList;

import com.team2568.frc2022.registers.ResetableRegister;
import com.team2568.frc2022.registers.StoppableRegister;
import com.team2568.frc2022.registers.UpdateRegister;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ILooper instances manage a list of runnables that share a common period.
 * Runnables can be added dynamically when inactive. However, runnables cannot
 * be removed. Register a register to a looper if a runnable here is the source
 * (FSMs are output latched).
 * 
 * @author Ryan Chaiyakul
 */
public class ILooper {
	// Flags
	private boolean mActive = false;
	private final Object mLoopLock = new Object();

	// Notifier
	private Notifier mNotifier;
	private String mName;
	private double kPeriod;

	// Registered objects
	private ArrayList<Runnable> mRunnables = new ArrayList<Runnable>();

	private ArrayList<UpdateRegister<?>> mUpdateRegisters = new ArrayList<UpdateRegister<?>>();
	private ArrayList<ResetableRegister<?>> mResetableRegisters = new ArrayList<ResetableRegister<?>>();
	private ArrayList<StoppableRegister<?>> mStoppableRegisters = new ArrayList<StoppableRegister<?>>();

	// Runnable instance that executes onRunnablefor every loop and updates
	// registers
	private final Runnable mDefaultRunnable = new Runnable() {
		@Override
		public void run() {
			if (mActive) {
				double mStart = Timer.getFPGATimestamp();

				synchronized (mLoopLock) {
					for (Runnable runnable : mRunnables) {
						runnable.run();
					}
				}
				updateRegisters();

				if (Registers.kTelemetry.get()) {
					SmartDashboard.putNumber(mName, Timer.getFPGATimestamp() - mStart);
				}
			}
		}
	};

	public ILooper(String name) {
		this(name, Constants.kDefaultPeriod);
	}

	public ILooper(String name, double period) {
		this.kPeriod = period;
		this.mName = name;

		this.mNotifier = new Notifier(mDefaultRunnable);
		mNotifier.setName(name);
	}

	/**
	 * Register loops to be controlled by this looper
	 * 
	 * @param runnables
	 */
	public void registerRunnables(Runnable... runnables) {
		if (!isActive()) {
			synchronized (mLoopLock) {
				for (Runnable runnable : runnables) {
					mRunnables.add(runnable);
				}
			}
		}
	}

	/**
	 * Register registers to be updated by this looper
	 * 
	 * @param registers
	 */
	public void registerUpdateRegisters(UpdateRegister<?>... registers) {
		if (!isActive()) {
			for (UpdateRegister<?> register : registers) {
				mUpdateRegisters.add(register);
			}
		}
	}

	/**
	 * Register registers to be reset by this looper
	 * 
	 * @param registers
	 */
	public void registerResetableRegisters(ResetableRegister<?>... registers) {
		if (!isActive()) {
			for (ResetableRegister<?> register : registers) {
				mResetableRegisters.add(register);
			}
		}
		registerUpdateRegisters(registers);
	}

	/**
	 * Register registers to be stopped by this looper
	 * 
	 * @param registers
	 */
	public void registerStoppableRegisters(StoppableRegister<?>... registers) {
		if (!isActive()) {
			for (StoppableRegister<?> register : registers) {
				mStoppableRegisters.add(register);
			}
		}
		registerResetableRegisters(registers);
	}

	/**
	 * Starts periodic notifier and runs one cycle with the LooperState as RESET
	 */
	public void start() {
		if (!isActive()) {
			resetRegisters();
			updateRegisters();

			mNotifier.startPeriodic(kPeriod);
			mActive = true;
		}
	}

	/**
	 * Stops periodic notifier and executes one cycle with the LooperState as STOP
	 */
	public void stop() {
		if (isActive()) {
			mNotifier.stop();
			mActive = false;

			stopRegisters();
			updateRegisters();

			step();
		}
	}

	/**
	 * Stops and restarts the notifier and registers. Will just start if it is
	 * inactive.
	 */
	public void reset() {
		if (isActive()) {
			stop();
			start();
		} else {
			start();
		}
	}

	/**
	 * Executes update for every updatable register registered to this looper
	 */
	private void updateRegisters() {
		for (UpdateRegister<?> updateRegister : mUpdateRegisters) {
			updateRegister.update();
		}
	}

	/**
	 * Executes reset for every resetable register registered to this looper
	 */
	private void resetRegisters() {
		for (ResetableRegister<?> updateRegister : mResetableRegisters) {
			updateRegister.reset();
		}
	}

	/**
	 * Executes stop for every stoppable register registered to this looper
	 */
	private void stopRegisters() {
		for (StoppableRegister<?> updateRegister : mStoppableRegisters) {
			updateRegister.stop();
		}
	}

	/**
	 * Execute a single compute and update cycle.
	 */
	private void step() {
		for (Runnable runnable : mRunnables) {
			runnable.run();
		}

		updateRegisters();
	}

	/**
	 * Self explanatory
	 * 
	 * @return
	 */
	public boolean isActive() {
		return mActive;
	}
}
