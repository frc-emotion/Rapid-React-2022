package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax mHood;
    private WPI_TalonFX mL, mR;
    private DigitalInput mLimit;

    public Shooter() {
        mHood = new CANSparkMax(Constants.SHOOTER_HOOD_PORT, MotorType.kBrushless);
        mL = new WPI_TalonFX(Constants.SHOOTER_LEFT_PORT);
        mL.setInverted(InvertType.InvertMotorOutput);
        mR = new WPI_TalonFX(Constants.SHOOTER_RIGHT_PORT);
        mR.setInverted(InvertType.None);

        mL.setNeutralMode(NeutralMode.Coast);
        mR.setNeutralMode(NeutralMode.Coast);

        mR.follow(mL); // Do not know if it has to be inverted yet

        mL.config_kP(0, Constants.SHOOTER_KP);
        mL.config_kI(0, Constants.SHOOTER_KI);
        mL.config_kD(0, Constants.SHOOTER_KD);
        mL.config_kF(0, Constants.SHOOTER_KF);

        mLimit = new DigitalInput(Constants.SHOOTER_LIMIT_PORT);
    }

    /**
     * Start PID feedback loop
     */
    public void spin() {
        // mL.set(TalonFXControlMode.Velocity, toTicks(Constants.SHOOTER_TARGET_RPM));
        mL.set(0.55);

    }

    /**
     * Stop all motors
     */
    public void stop() {
        mHood.stopMotor();
        mL.stopMotor();
    }

    /**
     * Manually control the hood for testing purposes.
     * 
     * WARNING: Limit switch not coded in as the direction of the hood has not been
     * tested yet
     */
    public void teleopHood() {
        mHood.set(Constants.SHOOTER_TELEOP_SPEED * Robot.operatorController.getLeftY());
    }

    public void run() {
        if (Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            spin();
        } else if (Math.abs(Robot.operatorController.getLeftY()) >= Constants.TRIGGER_THRESHOLD) {
            teleopHood();
        } else {
            stop();
        }

        updateDashboard(); // Comment out if laggy
    }

    /**
     * Set the hood to output the velocity at a specified angle in degrees
     * 
     * 
     * TODO: Need to find relationship between encoder position and output angle
     * 
     * @param angle
     */
    public void setHoodAngle(double angle) {
    }

    /**
     * Get the current hood angle in terms of output angle
     * 
     * TODO: Look above
     * 
     * @return
     */
    public double getHoodAngle() {
        return 0;
    }

    /**
     * Checks whether the current RPM is within the threshold for shooting
     * 
     * @return
     */
    public boolean atTarget() {
        return Math.abs(Constants.SHOOTER_TARGET_RPM - getRPM()) < Constants.SHOOTER_THRESHOLD_RPM;
    }

    /**
     * Returns the RPM of the encoder
     * 
     * @return
     */
    public double getRPM() {
        return toRPM(mL.getSelectedSensorVelocity());
    }

    /**
     * Converts (ticks/100ms) to a more standard (revolutions/min)
     * 
     * 2048 TalonFX ticks = 1 Revolution
     * 100 ms = 0.100 seconds
     * 0.1 seconds = 0.00167 minutes
     * 
     * @param ticks
     * @return
     */
    private double toRPM(double ticks) {
        return ticks / 2048 * 600;
    }

    /**
     * Converts (revolutions/min) to (ticks/100ms)
     * 
     * 2048 TalonFX ticks = 1 Revolution
     * 100 ms = 0.100 seconds
     * 0.1 seconds = 0.00167 minutes
     * 
     * @param rpm
     * @return
     */
    private double toTicks(double rpm) {
        return rpm * 2048 / 600;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("ShooterRPM", getRPM());
        SmartDashboard.putNumber("HoodEncoder", mHood.getEncoder().getPosition() / 4096);
        SmartDashboard.putBoolean("ShooterSwitch", mLimit.get());
    }
}
