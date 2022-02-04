package com.team2568.frc2022;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.team2568.frc2022.registers.Register;
import com.team2568.frc2022.registers.SetOnceRegister;
import com.team2568.frc2022.registers.StringToValue;

/**
 * Declare global registers within this class as final constants. This class may
 * be generated later.
 */
public class Registers {
	// Global Registers

	public static final SetOnceRegister<Boolean> kTelemetry = new SetOnceRegister<Boolean>(StringToValue.kBoolean);
	public static final SetOnceRegister<Boolean> kReal = new SetOnceRegister<Boolean>(StringToValue.kBoolean);

	// Subsystem Teleop State Registers

	public static final List<Register<?>> kRegisters = Collections.unmodifiableList(Arrays.asList());
}