package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax mHood;
    private WPI_TalonFX mL, mR;
    private DigitalInput mLimit;

    private Macro target_macro = Macro.UpAgainst;

    private boolean reached_target = false; // Flag to determine if the shooter was ever at target rpm

    private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS,
            Constants.SHOOTER_KV, Constants.SHOOTER_KA);

    /**
     * Custom enum which stores the corresponding rpm and angle for each position
     */
    public enum Macro {
        UpAgainst(Constants.SHOOTER_RPM_UP_AGAINST, Constants.SHOOTER_ANGLE_UP_AGAINST),
        CargoLine(Constants.SHOOTER_RPM_CARGO_LINE, Constants.SHOOTER_ANGLE_CARGO_LINE),
        ClosePad(Constants.SHOOTER_RPM_CLOSE_PAD, Constants.SHOOTER_ANGLE_CLOSE_PAD),
        FarPad(Constants.SHOOTER_RPM_FAR_PAD, Constants.SHOOTER_ANGLE_FAR_PAD),
        Testing(0, 0);

        public final double target_rpm, target_angle;

        private Macro(double target_rpm, double target_angle) {
            this.target_rpm = target_rpm;
            this.target_angle = target_angle;
        }
    }

    public Shooter() {
        mL = new WPI_TalonFX(Constants.SHOOTER_LEFT_PORT);
        mR = new WPI_TalonFX(Constants.SHOOTER_RIGHT_PORT);

        mL.configFactoryDefault();
        mR.configFactoryDefault();

        mL.setInverted(InvertType.InvertMotorOutput);
        mR.setInverted(InvertType.None);

        mL.setNeutralMode(NeutralMode.Coast);
        mR.setNeutralMode(NeutralMode.Coast);

        mL.configVoltageCompSaturation(Constants.SHOOTER_NOMINAL_VOLTAGE);
        mL.enableVoltageCompensation(true);
        mR.configVoltageCompSaturation(Constants.SHOOTER_NOMINAL_VOLTAGE);
        mR.enableVoltageCompensation(true);

        mR.follow(mL); // Inversion set before

        mL.config_kP(0, Constants.SHOOTER_KP);
        mL.config_kD(0, Constants.SHOOTER_KD);

        mHood = new CANSparkMax(Constants.SHOOTER_HOOD_PORT, MotorType.kBrushless);

        mHood.setInverted(true);

        mHood.getEncoder().setPositionConversionFactor(Constants.SHOOTER_REV_TO_ANGLE);

        SparkMaxPIDController controller = mHood.getPIDController();

        controller.setP(Constants.SHOOTER_HOOD_KP);
        controller.setI(Constants.SHOOTER_HOOD_KI);
        controller.setD(Constants.SHOOTER_HOOD_KD);

        mLimit = new DigitalInput(Constants.SHOOTER_LIMIT_PORT);

        SmartDashboard.putNumber("ShooterTestRPM", 0);
        SmartDashboard.putNumber("ShooterTestAngle", 0);
    }

    /**
     * Teleop function
     */
    public void run() {
        if (Robot.operatorController.getRightTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            spinUp();
        } else if (Robot.operatorController.getAButton()) {
            shoot();
        } else if (Math.abs(Robot.operatorController.getRightY()) >= Constants.JOYSTICK_THRESHOLD) {
            teleopHood();
        } else if (Robot.operatorController.getPOV() != -1) {
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    setMacro(Macro.UpAgainst);
                    break;
                case 90:
                    // Right
                    setMacro(Macro.ClosePad);
                    break;
                case 180:
                    // Down
                    setMacro(Macro.CargoLine);
                    break;
                case 270:
                    // Left
                    setMacro(Macro.FarPad);
                    break;
            }
            goToMacro();
            // TESTING ONLY COMMENT OUT DURING COMPETITION
        } else if (Robot.operatorController.getBackButton()) {
            setMacro(Macro.Testing);
        } else {
            stop();
        }

        updateDashboard();
    }

    /**
     * Stop all motors and callibrate if at limit switch
     */
    public void stop() {
        if (atLimit()) {
            callibrate();
        }

        mHood.stopMotor();
        mL.stopMotor();

        reached_target = false;
    }

    /**
     * Zero hood encoder
     */
    private void callibrate() {
        mHood.getEncoder().setPosition(0);
    }

    /**
     * Unconditionally spin the shooter and indexer once the target is reached at
     * least once
     */
    public void shoot() {
        if (atRPM()) {
            reached_target = true;
        }

        if (reached_target) {
        //ADD INDEXER CODE 
        }
    }

    /**
     * Manually control the hood for testing purposes.
     * 
     */
    public void teleopHood() {
        double joystick = -Robot.operatorController.getRightY(); // Up is negative for Y

        if (joystick < 0 && atLimit()) {
            callibrate();
        } else {
            mHood.set(Constants.SHOOTER_HOOD_SPEED * joystick);
        }

    }

    /**
     * Set the local target macro to the passed macro
     * 
     * @param target_macro
     */
    public void setMacro(Macro target_macro) {
        this.target_macro = target_macro;
    }

    /**
     * Moves the hood to the angle specified by the local target macro
     */
    public void goToMacro() {
        if (target_macro == Macro.Testing) {
            setHoodAngle(SmartDashboard.getNumber("ShooterTestAngle", 0));
        } else {
            setHoodAngle(target_macro.target_angle);
        }
    }

    /**
     * Set the hood to output the velocity at a specified angle in degrees
     * 
     * 
     * @param angle the requested set point
     */
    private void setHoodAngle(double angle) {
        mHood.getPIDController().setReference(
                MathUtil.clamp(angle, Constants.SHOOTER_HOOD_MIN, Constants.SHOOTER_HOOD_MAX),
                ControlType.kPosition);
    }

    /**
     * Spin up the motor to the locally stored rpm
     */
    public void spinUp() {
          spinAt(1700);
    }

    /**
     * Start the shooter with the setpoint provided
     * 
     * @param rpm target rpm
     */
    private void spinAt(double rpm) {
        mL.set(ControlMode.Velocity, toNative(rpm),
                DemandType.ArbitraryFeedForward,
                mFeedForward.calculate(rpm) / Constants.SHOOTER_NOMINAL_VOLTAGE);
    }

    /**
     * Checks whether the limit switch is triggered
     * 
     * @return True if the hood is at the limit switch
     */
    public boolean atLimit() {
        return !mLimit.get();
    }

    /**
     * Get the current hood angle in terms of output angle
     * 
     * 
     * @return the current hood angle
     */
    public double getHoodAngle() {
        return mHood.getEncoder().getPosition() + Constants.SHOOTER_STARTING_ANGLE;
    }

    /**
     * Checks if the shooter rpm is at the current target rpm
     * 
     * @return True if the rpm is within the threshold
     */
    public boolean atRPM() {
        return Math.abs(getRPM() - target_macro.target_rpm) < Constants.SHOOTER_THRESHOLD_RPM;
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

    /**
     * Converts (revolutions/min) to (ticks/100ms)
     * 
     * 2048 TalonFX ticks = 1 Revolution
     * 100 ms = 0.100 seconds
     * 0.1 seconds = 0.00167 minutes
     * 
     * @param ticks
     * @return
     */
    private double toNative(double rpm) {
        return (rpm * 2048) / 600;
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("ShooterRPM", getRPM());
        SmartDashboard.putNumber("HoodAngle", getHoodAngle());
        SmartDashboard.putString("ShooterState", target_macro.toString());
    }
}