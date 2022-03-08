package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax mHood;
    private WPI_TalonFX mL, mR;
    private DigitalInput mLimit;

    private double speed = 0.35; 

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

        SparkMaxPIDController controller = mHood.getPIDController();

        controller.setP(Constants.SHOOTER_HOOD_KP);
        controller.setP(Constants.SHOOTER_HOOD_KI);
        controller.setP(Constants.SHOOTER_HOOD_KD);

        mLimit = new DigitalInput(Constants.SHOOTER_LIMIT_PORT);
        SmartDashboard.putNumber("ShooterSpeed", speed);
    }

    /**
     * Start PID feedback loop
     */
    public void spin() {
        // mL.set(TalonFXControlMode.Velocity, toTicks(Constants.SHOOTER_TARGET_RPM));
        mL.set(speed);

    }

    /**
     * Stop all motors
     */
    public void stop() {
        if (atLimit()) {
            callibrate();
        }

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
        double joystick = Robot.operatorController.getLeftY();

        if (joystick > 0 && atLimit()) {
            callibrate();
        } else {
            mHood.set(Constants.SHOOTER_TELEOP_SPEED * joystick);
        }

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
     * @param angle
     */
    public void setHoodAngle(double angle) {
        mHood.getPIDController().setReference(
                hoodAngletoTick(-MathUtil.clamp(angle, Constants.SHOOTER_HOOD_MIN, Constants.SHOOTER_HOOD_MAX)),
                ControlType.kPosition);
    }

    /**
     * Get the current hood angle in terms of output angle
     * 
     * 
     * @return
     */
    public double getHoodAngle() {
        return tickToHoodAngle(-mHood.getEncoder().getPosition());
    }

    private double tickToHoodAngle(double ticks) {
        return ticks * 0.416;
    }

    private double hoodAngletoTick(double angle) {
        return angle / 0.416;
    }

    /**
     * Stop hood and zero encoder
     */
    private void callibrate() {
        mHood.stopMotor();
        mHood.getEncoder().setPosition(0);
    }

    public boolean atLimit() {
        return !mLimit.get();
    }

    /**
     * Checks whether the current RPM is within the threshold for shooting
     * 
     * @return
     */
    public boolean atRPM() {
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
    private double toRPM(double ticks_per_time) {
        return ticks_per_time / 2048 * 600;
    }

    public void updateDashboard() {
        speed = SmartDashboard.getNumber("ShooterSpeed", 0.35);
        SmartDashboard.putNumber("ShooterRPM", getRPM());
        SmartDashboard.putNumber("HoodAngle", getHoodAngle());
    }
}
