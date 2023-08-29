package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.TreeMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Distance;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.LimeLight;
import frc.robot.util.dashboard.TabManager;
import frc.robot.util.dashboard.TabManager.SubsystemTab;

public class Shooter extends SubsystemBase {
    private CANSparkMax mHood;
    // private CANSparkMax mL, mR;
    private CANSparkMax shooter;
    private DigitalInput mLimit;
    private LimeLight LL;
    private Distance distance;

    public double distanceToTarget;
    public TreeMap<Double, Pair<Double, Double>> MacroTable;
    public InterpolatingTreeMap ShooterTable;

    private Macro target_macro = Macro.FenderHigh;

    private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(Constants.SHOOTER_KS,
            Constants.SHOOTER_KV, Constants.SHOOTER_KA);

    public NetworkTableEntry shooterRPMGraph;
    public NetworkTableEntry shooterStateEntry;
    public NetworkTableEntry hoodAngle;

    /**
     * Custom enum which stores the corresponding rpm and angle for each position
     */
    public enum Macro {
        FenderLow(Constants.SHOOTER_RPM_FENDER_LOW, Constants.SHOOTER_ANGLE_FENDER_LOW),
        FenderHigh(Constants.SHOOTER_RPM_FENDER_HIGH, Constants.SHOOTER_ANGLE_FENDER_HIGH),
        CargoLine(Constants.SHOOTER_RPM_CARGO_LINE, Constants.SHOOTER_ANGLE_CARGO_LINE),
        OutsideTarmac(Constants.SHOOTER_RPM_OUTSIDE_TARMAC, Constants.SHOOTER_ANGLE_OUTSIDE_TARMAC),
        ClosePad(Constants.SHOOTER_RPM_CLOSE_PAD, Constants.SHOOTER_ANGLE_CLOSE_PAD),
        FarPad(Constants.SHOOTER_RPM_FAR_PAD, Constants.SHOOTER_ANGLE_FAR_PAD),
        Testing(1600, 4.5);

        public final double target_rpm, target_angle;

        private Macro(double target_rpm, double target_angle) {
            this.target_rpm = target_rpm;
            this.target_angle = target_angle;
        }
    }

    public Shooter() {
        // mL = new CANSparkMax(Constants.SHOOTER_LEFT_PORT, MotorType.kBrushless);
        shooter = new CANSparkMax(Constants.SHOOTER_PORT, MotorType.kBrushless);
        // mR = new CANSparkMax(Constants.SHOOTER_RIGHT_PORT, MotorType.kBrushless);

        distance = new Distance(Constants.MOUNTING_HEIGHT, Constants.MOUNTING_ANGLE, Constants.REFERENCE_HEIGHT);
        LL = new LimeLight();

        MacroTable = new TreeMap<Double, Pair<Double, Double>>();
        MacroTable.put(20.0, new Pair<>(1600.0, 11.5));
        MacroTable.put(20.0, new Pair<>(1750.0, 18.0));

        ShooterTable = new InterpolatingTreeMap(MacroTable);
        ShooterTable.interpolate(1);

        // shooter.configFactoryDefault();
        // mR.configFactoryDefault();

        // shooter.setInverted(InvertType.InvertMotorOutput);
        // mR.setInverted(InvertType.None);

        // shooter.setInverted(true);
        // mR.setInverted(false);

        // shooter.setNeutralMode(NeutralMode.Coast);
        // mR.setNeutralMode(NeutralMode.Coast);

        shooter.setIdleMode(IdleMode.kBrake);
        shooter.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
        shooter.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
        // mR.setIdleMode(IdleMode.kBrake);

        // shooter.configVoltageCompSaturation(Constants.SHOOTER_NOMINAL_VOLTAGE);
        // shooter.enableVoltageCompensation(true);
        // mR.configVoltageCompSaturation(Constants.SHOOTER_NOMINAL_VOLTAGE);
        // mR.enableVoltageCompensation(true);

        // shooter.enableVoltageCompensation(Constants.SHOOTER_NOMINAL_VOLTAGE);
        // mR.enableVoltageCompensation(Constants.SHOOTER_NOMINAL_VOLTAGE);

        // mR.follow(mL); // Inversion set before

        // shooter.config_kP(0, Constants.SHOOTER_KP);
        // shooter.config_kD(0, Constants.SHOOTER_KD);

        shooter.getPIDController().setP(Constants.SHOOTER_KP);
        shooter.getPIDController().setD(Constants.SHOOTER_KD);

        mHood = new CANSparkMax(Constants.SHOOTER_HOOD_PORT, MotorType.kBrushless);

        mHood.restoreFactoryDefaults();

        mHood.setInverted(true);
        mHood.enableVoltageCompensation(Constants.SHOOTER_HOOD_NOMINAL_VOLTAGE);
        mHood.setIdleMode(IdleMode.kBrake);

        SparkMaxPIDController controller = mHood.getPIDController();

        controller.setP(Constants.SHOOTER_HOOD_KP, 0);
        controller.setD(Constants.SHOOTER_HOOD_KD, 0);

        mLimit = new DigitalInput(Constants.SHOOTER_LIMIT_PORT);

        // TESTING ONLY COMMENT OUT DURING COMPETITION
        SmartDashboard.putNumber("ShooterTestRPM", 1600);
        SmartDashboard.putNumber("ShooterTestAngle", 4.5);

        initShuffleboard();

    }

    @Override
    public void periodic() {
        // Put SmartDashboard values here

        if (atLimit()) {
            calibrate();
        }
        distanceToTarget = distance.getDistance(LL.getTy());
        updateShuffleboard();
    }

    /**
     * Teleop function
     */
    public void run() {
        if (Robot.operatorController.getRightTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            shooter.set(0.2);
            //shoot();
            /**
             * if (Robot.operatorController.getPOV() == -1){
             * spinUpTable();
             * }else{
             * shoot()
             * }
             */
        } else if (Math.abs(Robot.operatorController.getLeftY()) >= Constants.JOYSTICK_THRESHOLD) {
            teleopHood();
        } else if (Robot.operatorController.getBButton()) {
            shooter.set(-0.1);
        } else if (Robot.operatorController.getPOV() != -1) {
            switch (Robot.operatorController.getPOV()) {
                case 0:
                    // Up
                    setMacro(Macro.FenderHigh);
                    break;
                case 90:
                    // Right
                    setMacro(Macro.FenderLow);
                    break;
                case 180:
                    // Down
                    setMacro(Macro.ClosePad);
                    break;
                case 270:
                    // Left
                    setMacro(Macro.OutsideTarmac);
                    break;
            }
            goToMacro();
        } else if (Robot.operatorController.getBackButton()) {
            aimAtTarget();
            // setMacro(Macro.Testing);
            // goToMacro();
        } else {
            stop();
        }

    }

    public void stopHood() {
        mHood.stopMotor();
    }

    /**
     * Stop all motors and calibrate if at limit switch
     */
    public void stop() {
        mHood.stopMotor();
        shooter.stopMotor();
    }

    /**
     * Zero hood encoder
     */
    private void calibrate() {
        mHood.stopMotor();
        mHood.getEncoder().setPosition(0);
    }

    /**
     * Unconditionally spin the shooter and indexer once the target is reached at
     * least once and A button is pressed
     */
    public void shoot() {
        spinUp();

        /*
         * if (Robot.operatorController.getAButton()) {
         * isReady = true;
         * }
         */

        // TRIGGER COMMAND ON A BUTTON//Robot.indexer.indexForward();
        // }
    }

    /**
     * Spin up the motor to the locally stored rpm
     */
    public void spinUp() {
        // TESTING ONLY COMMENT OUT DURING COMPETITION
        if (target_macro == Macro.Testing) {
            spinAt(SmartDashboard.getNumber("ShooterTestRPM", 1600));
        } else {
            spinAt(-target_macro.target_rpm);
        }
    }

    /**
     * Spin up the motor based on distance from the target
     */
    public void spinUpTable() {
        spinAt(ShooterTable.getRPM(distanceToTarget));
    }

    /**
     * Start the shooter with the setpoint provided
     * 
     * @param rpm target rpm
     */
    private void spinAt(double rpm) {
        // shooter.set(ControlMode.Velocity, toNative(rpm),
        //         DemandType.ArbitraryFeedForward,
        //         mFeedForward.calculate(rpm / 60) / Constants.SHOOTER_NOMINAL_VOLTAGE);
        //shooter.getPIDController().setReference(rpm, ControlType.kVelocity);
        shooter.set(0.1);
    }

    /**
     * Manually control the hood for testing purposes.
     * 
     */
    public void teleopHood() {
        double joystick = -Robot.operatorController.getLeftY(); // Up is negative for Y
        if (joystick < 0 && atLimit()) {
            calibrate();
        } else if (getHoodAngle() > 40 && joystick > 0) {
            mHood.stopMotor();
        } else {
            if (getHoodAngle() < 2 && joystick < 0) {
                mHood.set(Constants.SHOOTER_HOOD_ZERO_SPEED * joystick);
            } else if (getHoodAngle() > 38 && joystick > 0) {
                mHood.set(Constants.SHOOTER_HOOD_ZERO_SPEED * joystick);
            } else {
                mHood.set(Constants.SHOOTER_HOOD_SPEED * joystick);
            }
        }
    }

    public void autoZero() {
        if (!atLimit()) {
            mHood.set(-0.3);
        } else if (atLimit()) {
            calibrate();
        }
    }

    public void autoShoot(boolean ready, double rpm, double angle) {
        setHoodAngle(angle);
        spinAt(rpm);

        if (getRPM() > rpm - 5) {
            ready = true;
        }
    }

    public void autoHood(double angle) {
        setHoodAngle(angle);
    }

    public void autoShootTime() {
        spinAt(Constants.SHOOTER_RPM_AUTO);
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
        // TESTING ONLY COMMENT OUT DURING COMPETITION
        if (target_macro == Macro.Testing) {
            setHoodAngle(SmartDashboard.getNumber("ShooterTestAngle", 4.5));
        } else {
            setHoodAngle(target_macro.target_angle);
        }
    }

    /**
     * Change hoodAngle when aligned
     * Use Robot Pose (LL Correction) to set hoodAngle automatically.
     */
    public void aimAtTarget() {
        setHoodAngle(ShooterTable.getAngle(distanceToTarget));
    }

    /**
     * Set the hood to output the velocity at a specified angle in degrees
     * 
     * 
     * @param angle the requested set point
     */
    public void setHoodAngle(double angle) {
        mHood.getPIDController().setReference(
                MathUtil.clamp(angle, Constants.SHOOTER_HOOD_MIN, Constants.SHOOTER_HOOD_MAX)
                        / Constants.SHOOTER_REV_TO_ANGLE,
                ControlType.kPosition);
    }

    /**
     * Get the current hood angle in terms of output angle
     * 
     * 
     * @return the current hood angle
     */
    public double getHoodAngle() {
        return mHood.getEncoder().getPosition() * Constants.SHOOTER_REV_TO_ANGLE + Constants.SHOOTER_STARTING_ANGLE;
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
        return toRPM(shooter.getEncoder().getPosition());
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
        return (ticks_per_time / 2048) * 600;
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

    /**
     * Checks whether the limit switch is triggered
     * 
     * @return True if the hood is at the limit switch
     */
    public boolean atLimit() {
        return !mLimit.get();
    }

    /**
     * Checks if shooter aims by Macro/LookupTable
     * @return shooter state with regards to boolean
     */
    public String shooterState(boolean shootByInterpolation) {
        return shootByInterpolation ? "Interpolating Shots" : target_macro.toString();
    }

    private void initShuffleboard() {
        ShuffleboardTab shooterData = TabManager.getInstance().accessTab(SubsystemTab.SHOOTER);
        shooterRPMGraph = TabManager.getInstance().addWidget(shooterData, BuiltInWidgets.kTextView, "RPM", 0,
                new int[] { 0, 0 }, new int[] { 4, 4 });
        hoodAngle = TabManager.getInstance().addWidget(shooterData, BuiltInWidgets.kTextView, "Hood Angle", 0,
                new int[] { 4, 0 }, new int[] { 2, 2 });
        shooterStateEntry = TabManager.getInstance().addWidget(shooterData, BuiltInWidgets.kTextView, "Macro State",
                shooterState(false), new int[] { 6, 0 }, new int[] { 2, 2 });
    }

    private void updateShuffleboard() {
        shooterRPMGraph.setDouble(getRPM());
        hoodAngle.setDouble(getHoodAngle());
        shooterStateEntry.setString(shooterState(true));
    }
}