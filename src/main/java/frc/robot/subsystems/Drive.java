package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drive extends SubsystemBase {
    CANSparkMax lsparkA = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax lsparkB = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax lsparkC = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax rsparkA = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax rsparkB = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax rsparkC = new CANSparkMax(10, MotorType.kBrushless);

    private final MotorControllerGroup leftGroup = new MotorControllerGroup(lsparkA, lsparkB, lsparkC);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(rsparkA, rsparkB, rsparkC);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    private final RelativeEncoder leftEncoder = lsparkA.getEncoder();
    private final RelativeEncoder rightEncoder = rsparkA.getEncoder();

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final DifferentialDriveOdometry odometry;

    private boolean invert;

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

        rsparkA.setInverted(true);
        rsparkB.setInverted(true);
        rsparkC.setInverted(true);

        // Velocity Conversions
        leftEncoder.setVelocityConversionFactor((1/Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);
        rightEncoder.setVelocityConversionFactor((1/Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0) / 60);

        // Position Conversions
        leftEncoder.setPositionConversionFactor((1/Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0));
        rightEncoder.setPositionConversionFactor((1/Constants.kDriveGearRatio) * 2 * Math.PI * Units.inchesToMeters(3.0));

        //Zero Encoders
        resetEncoders();
        // Sets the robot Position in a 2D Space
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        this.invert = false;
    }



    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition());

    }

    public void updatedField(){
        //Used Field 2D on shuffleboard when/if basic odometry works
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
        return gyro.getRate();
    }

    public void run(){
        if(Robot.driverController.getRightStickButton()){
            invert = !invert;
        }
        teleopTank();
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
        double drivePower = 0.6;
        if (Robot.driverController.getLeftBumper())
            drivePower = 0.3;
        else if (Robot.driverController.getRightBumper())
            drivePower = 0.9;

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

}