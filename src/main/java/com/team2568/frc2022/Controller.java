package com.team2568.frc2022;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Cleans up the API to the XboxController by applying a deadzones (Hand fixing
 * fixed on WPILIB 2022). In the future, inputs may be buffered so that race
 * conditions will not occur during the compute phase of the subsystems.
 * 
 * @author Ryan Chaiyakul
 */
public class Controller extends XboxController {

    public static Controller driveController = new Controller(Constants.kDriveControllerPort);
    public static Controller operatController = new Controller(Constants.kOperatorControllerPort);

    private Controller(int port) {
        super(port);
    }

    /**
     * Returns whether the left trigger is pressed beyond the deadzone
     */
    public boolean getLeftTrigger() {
        return getTriggerDeadzone(getLeftTriggerAxis());
    }

    /**
     * Returns whether the right trigger is pressed beyond the deadzone
     */
    public boolean getRightTrigger() {
        return getTriggerDeadzone(getRightTriggerAxis());
    }

    /**
     * Private method that checks whether the axis is past the deadzone
     * 
     * @param triggerAxis
     * @return
     */
    private boolean getTriggerDeadzone(double triggerAxis) {
        return Constants.kTriggerThreshold <= Math.abs(triggerAxis);
    }

    /**
     * Private method that removes joystick noise around 0
     * 
     * @param deadzone
     * @return
     */
    private double getDeadzone(double deadzone) {
        if (Math.abs(deadzone) < Constants.kJoystickDeadzone) {
            return 0;
        }
        return deadzone;
    }

    public double getLeftDeadzoneX() {
        return getDeadzone(getLeftX());
    }

    public double getRightDeadzoneX() {
        return getDeadzone(getRightX());
    }

    public double getLeftDeadzoneY() {
        return getDeadzone(getLeftY());
    }

    public double getRightDeadzoneY() {
        return getDeadzone(getRightY());
    }
}
