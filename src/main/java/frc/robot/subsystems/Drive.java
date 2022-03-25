package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends SubsystemBase {
    CANSparkMax lsparkA = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[0], MotorType.kBrushless);
    CANSparkMax lsparkB = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[1], MotorType.kBrushless);
    CANSparkMax lsparkC = new CANSparkMax(Constants.DRIVE_LEFT_PORTS[2], MotorType.kBrushless);
    CANSparkMax rsparkA = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[0], MotorType.kBrushless);
    CANSparkMax rsparkB = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[1], MotorType.kBrushless);
    CANSparkMax rsparkC = new CANSparkMax(Constants.DRIVE_RIGHT_PORTS[2], MotorType.kBrushless);

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(lsparkA, lsparkB, lsparkC);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rsparkA, rsparkB, rsparkC);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    // USEFUL FOR NEW ROBOT (IF NOT CHARACTERIZED)
    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(3),
            Constants.kDriveGearRatio,
            6.9,
            55.9,
            Units.inchesToMeters(3),
            0.69,

            // The standard deviations for measurement noise:
            // x and y: 0.001 m
            // heading: 0.001 rad
            // l and r velocity: 0.1 m/s
            // l and r position: 0.005 m
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    private final RelativeEncoder leftEncoder = lsparkA.getEncoder();
    private final RelativeEncoder rightEncoder = rsparkA.getEncoder();

    // Custom Devices Include CANSparkMax Integrated Encoders
    int m1 = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [1]");

    SimDouble m1_encoderPos = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m1, "Position"));
    SimDouble m1_encoderVel = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m1, "Velocity"));

    int m2 = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [4]");

    SimDouble m2_encoderPos = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m1, "Position"));
    SimDouble m2_encoderVel = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m1, "Velocity"));

    // For inital sim testing
    private final AHRS gyro = new AHRS();

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    // private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    // private ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    private final DifferentialDriveOdometry odometry;

    private boolean invert;

    public static final Field2d m_field = new Field2d();

    public Drive() {

        ArrayList<CANSparkMax> sparkList = new ArrayList<CANSparkMax>() {
            {
                add(lsparkA);
                add(lsparkB);
                add(lsparkC);
                add(rsparkA);
                add(rsparkB);
                add(rsparkC);
            }
        };

        // Voltage Regulation
        for (CANSparkMax spark : sparkList) {
            spark.setSmartCurrentLimit(45);
            spark.setSecondaryCurrentLimit(45);
            spark.setIdleMode(IdleMode.kBrake);
        }

        lsparkA.setInverted(false);
        lsparkB.setInverted(false);
        lsparkC.setInverted(false);

        rsparkA.setInverted(true);
        rsparkB.setInverted(true);
        rsparkC.setInverted(true);

        // Velocity Conversions
        leftEncoder.setVelocityConversionFactor(
                (1 / Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
        rightEncoder.setVelocityConversionFactor(
                (1 / Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);

        // Position Conversions
        leftEncoder
                .setPositionConversionFactor((1 / Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0));
        rightEncoder
                .setPositionConversionFactor((1 / Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0));

        gyro.calibrate();

        // Zero Encoders
        resetEncoders();
        // Sets the robot Position in a 2D Space
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        SmartDashboard.putData("Field", m_field);

        this.invert = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", gyro.getYaw());

        m_field.setRobotPose(odometry.getPoseMeters());
        odometry.update(gyro.getRotation2d(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition());

        simPeriodic();
    }

    public void updatedField() {
        // Used Field 2D on shuffleboard when/if basic odometry works
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // returns in RPM, divide by gear ratio multiply by circumference and /60 to get
        // Meters per second
        // RPM --> MPS (meters per second)
        return new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity(),
                rightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftGroup.setVoltage(leftVolts);
        rightGroup.setVoltage(rightVolts);
        drive.feed();
    }

    public RelativeEncoder getLeftCanEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightCanEncoder() {
        return rightEncoder;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    public void run() {
        if (Robot.driverController.getXButtonPressed()) {
            invert = !invert;
        }
        if (Robot.driverController.getAButton()) {
            forward();
        } else {
            teleopTank();
        }

    }

    /**
     * TODO: Clean up turn code (easier to switch)
     * @param degrees angle to turn to (absolute)
     * @param lr counterclockwise or clockwise turn
     */
    public void gyroTurn(double degrees, boolean lr) {
        if (lr) {
            if (gyro.getYaw() < degrees) {
                drive.arcadeDrive(0, 0.3);
            }
            if (gyro.getYaw() > degrees) {
                drive.arcadeDrive(0, 0);
            }
        } else if (!lr) {
            if (gyro.getYaw() > degrees) {
                drive.arcadeDrive(0, -0.3);
            }
            if (gyro.getYaw() < degrees) {
                drive.arcadeDrive(0, 0);
            }
        }
    }

    public void autoforward(double speed) {
        drive.arcadeDrive(-speed, 0);
    }

    public void moveForward() {
        drive.arcadeDrive(Constants.DRIVE_FORWARD_SPEED, 0);
    }

    public void forward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        int sign = 1;
        if (invert) {
            sign *= -1;
        }

        double driveSpeed = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getLeftBumper())
            driveSpeed = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getRightBumper())
            driveSpeed = Constants.DRIVE_TURBO_POWER;

        drive.arcadeDrive(sign * driveSpeed, 0);
    }

    public void backward() {
        // LB and RB are used to change the driveSpeed during the match
        // Drive power constants might be correct
        double driveSpeed = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getLeftBumper())
            driveSpeed = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getRightBumper())
            driveSpeed = Constants.DRIVE_TURBO_POWER;

        drive.arcadeDrive(-driveSpeed, 0);
    }

    public void stop() {
        drive.arcadeDrive(0, 0);
    }

    public void teleopTank() {
        // constants to easily configure if drive is opposite
        int constR = 1, constL = 1;

        // Get vertical value of the joysticks
        double rAxis = Robot.driverController.getRightY();
        double lAxis = Robot.driverController.getLeftY();

        // Use a constant multiplier for +/- direction as the driveExponent could be
        // even and negate the sign
        if (rAxis < 0) {
            constR *= 1;
        } else if (rAxis > 0) {
            constR *= -1;
        }

        if (lAxis < 0) {
            constL *= 1;
        } else if (lAxis > 0) {
            constL *= -1;
        }

        // LB and RB are used to change the drivePower during the match
        double drivePower = Constants.DRIVE_REGULAR_POWER;
        if (Robot.driverController.getLeftBumper())
            drivePower = Constants.DRIVE_SLOW_POWER;
        else if (Robot.driverController.getRightBumper())
            drivePower = Constants.DRIVE_TURBO_POWER;

        // However driveExponent should be constant (Changeable by SmartDashboard)
        double driveExponent = SmartDashboard.getNumber("Drive Exponent", 1.8);

        // Use an exponential curve to provide fine control at low speeds but with a
        // high maximum speed
        double driveL = constL * drivePower * Math.pow(Math.abs(lAxis), driveExponent);
        double driveR = constR * drivePower * Math.pow(Math.abs(rAxis), driveExponent);

        // Our drivers prefer tankDrive
        // invert will switch R and L
        if (invert) {
            drive.tankDrive(-driveR, -driveL);
        } else {
            drive.tankDrive(driveL, driveR);

        }
    }

    public void simPeriodic() {

        driveSim.setInputs(lsparkA.get() * RobotController.getInputVoltage(),
                rsparkA.get() * RobotController.getInputVoltage());
        driveSim.update(0.02);

        m1_encoderPos.set(driveSim.getLeftPositionMeters());
        m1_encoderVel.set(driveSim.getLeftVelocityMetersPerSecond());
        m2_encoderPos.set(driveSim.getRightPositionMeters());
        m2_encoderVel.set(driveSim.getRightVelocityMetersPerSecond());

        angle.set(-driveSim.getHeading().getDegrees());

    }

}