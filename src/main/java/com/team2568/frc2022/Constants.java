package com.team2568.frc2022;

/**
 * Static collection of constants. Check the <a href=
 * "https://docs.google.com/spreadsheets/d/1i3ux4MvYPfBhqnBxSJhU4P-CUI5KfEUIO1yJBmXWYIk/edit?usp=sharing">
 * Google spreadsheet </a> for readable version.
 * 
 * @author Ryan Chaiyakul
 */
public class Constants {
	// Runnable Constants
	public static final double kDefaultPeriod = 0.01;

	// USB Ports
	public static final int kDriveControllerPort = 0;
	public static final int kOperatorControllerPort = 1;

	// Controller Constants
	public static final double kTriggerThreshold = 0.3;
	public static final double kJoystickDeadzone = 0.1;

	// Motor Constants

	// SparkMax Constants
	public static final int kSparkSmartCurrentLimit = 35;
	public static final int kSparkSecondaryCurrentLimit = 40;

	// CTRE Constants

	// TalonFX Constants
	public static final double kTalonSupplyCurrentLimit = 60;
	public static final double kTalonSupplyTriggerCurrent = 50;
	public static final double kTalonSupplyTriggerTime = 0.1;

	public static final double kTalonStatorCurrentLimit = 55;
	public static final double kTalonStatorTriggerCurrent = 45;
	public static final double kTalonStatorTriggerTime = 0.1;

	// CAN Ports

	// CANSparkMAXs
	public static final int[] kDriveLeftPorts = { 1, 2, 3 };
	public static final int[] kDriveRightPorts = { 4, 5, 6 };

	public static final int kIntakePort = 7;
	public static final int kShooterHoodPort = 8;
	// TalonFXs
	public static final int kShooterLeftPort = 9;
	public static final int kShooterRightPort = 10;

	public static final int kClimbLeftPort = 11;
	public static final int kClimbRightPort = 12;

	public static final int kIndexerPort = 13;

	// Pneumatic Ports
	public static final int kClimbLeftForwardPort = 1;
	public static final int kClimbLeftReversePort = 2;

	public static final int kClimbRightForwardPort = 3;
	public static final int kClimbRightReversePort = 4;

	public static final int kIntakeForwardPort = 5;
	public static final int kIntakeReversePort = 6;

}
