package com.team2568.frc2022;

/**
 * Static collection of constants. Check the google spreadsheet below for a more
 * readable version
 * 
 * https://docs.google.com/spreadsheets/d/1i3ux4MvYPfBhqnBxSJhU4P-CUI5KfEUIO1yJBmXWYIk/edit?usp=sharing
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
	public static final int kSparkCurrentLimit = 40;

	// TalonSRX Constants
	public static final int kTalonSRXCurrentLimit = 60;

	// CAN Ports

	// CANSparkMAXs
	public static final int[] kDriveLeftPorts = { 1, 2, 3 };
	public static final int[] kDriveRightPorts = { 4, 5, 6 };

	public static final int kIntakePort = 7;

	// TalonFXs
	public static final int kClimbLeftPort = 8;
	public static final int kClimbRightPort = 9;

	public static final int kShooterLeftPort = 10;
	public static final int kShooterRightPort = 11;
	public static final int kShooterHoodPort = 12;

	public static final int kIndexerPort = 13;

	// Pneumatic Ports
	public static final int kClimbLeftForwardPort = 1;
	public static final int kClimbLeftReversePort = 2;

	public static final int kClimbRightForwardPort = 3;
	public static final int kClimbRightReversePort = 4;

	public static final int kIntakeForwardPort = 5;
	public static final int kIntakeReversePort = 6;

}
