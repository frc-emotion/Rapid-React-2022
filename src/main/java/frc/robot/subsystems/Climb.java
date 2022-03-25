package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * @author Karan Thakkar
 */

public class Climb extends SubsystemBase {
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    public static DoubleSolenoid ActuatorR, ActuatorL;
    private boolean atMax;
    private boolean removeBounds;
    public double max;
    public double min;

    public static boolean hook;

    /**
     * Elevator Sim Testing
     * private final ElevatorSim m_elevatorSim =
     * new ElevatorSim(
     * DCMotor.getFalcon500(2),
     * 21.33,
     * 30,
     * 15,
     * 0.952,
     * 2,
     * VecBuilder.fill(0.01));
     * 
     * int climb11 = SimDeviceDataJNI.getSimDeviceHandle("Talon FX[11]/Integrated
     * Sensor");
     * 
     * SimDeviceSim climbSim = new SimDeviceSim("Talon FX[11]/Integrated Sensor");
     * 
     * private final SimDouble sim_Encoder_Pos = climbSim.getDouble("position");
     */
    public Climb() {
        // Init Double Solenoids, Falcons
        ActuatorL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMB_PNEUMATICS[0],
                Constants.CLIMB_PNEUMATICS[1]);

        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and Add Factory Default Settings
        TalonA.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));
        TalonB.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));

        TalonB.follow(TalonA);

        // Encoder Config
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        TalonA.config_kP(0, Constants.CLIMB_kP);
        TalonA.config_kI(0, Constants.CLIMB_kI);
        TalonA.config_kD(0, Constants.CLIMB_kD);

        // Output Encoder Values
        SmartDashboard.putNumber("ClimbMAX", (Constants.CLIMB_MAX_POS));
        SmartDashboard.putNumber("ClimbMIN", 0);

        atMax = false;
        removeBounds = false;
        ActuatorL.set(Value.kReverse);
        hook = false;

    }

    @Override
    public void periodic() {
        updateDash();
    }

    // Mainloop
    public void run() {
        double lJoystickPos = Robot.operatorController.getRightY();

        // Statements are Reversed, as the Joystick is reversed
        if (lJoystickPos < -Constants.JOYSTICK_THRESHOLD && !atMax) {
            TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
        } else if (lJoystickPos > Constants.JOYSTICK_THRESHOLD) {
            TalonA.set(Constants.CLIMB_MOTOR_SPEED);
        } else if (Robot.operatorController.getStartButtonPressed()) {
            ZeroEncoders();
        } else {
            TalonA.set(0);
        }

        if (Robot.operatorController.getYButtonPressed()) {
            ActuatorL.toggle();
        }

        // click right stick button to toggle bounds IF NEEDED
        if (Robot.operatorController.getRightStickButtonPressed() && !removeBounds) {
            removeBounds = !removeBounds;
        }

        Bounds();
    }

    /**
     * 
     * @return TalonFx Encoder Position (Encoder Units)
     */
    public double getPosition() {
        return TalonA.getSelectedSensorPosition();
    }

    public double getVel() {
        return TalonA.getSelectedSensorVelocity();
    }

    public void Bounds() {
        if (returnRevs() < -max) {
            atMax = true;
        } else {
            atMax = false;
        }
    }

    /** 
     * TalonFx Units Per Rev (NON QUADRATURE) = 2048 
     * 
     * */
    public double returnRevs() {
        return getPosition() / 2048;
    }

    private void ZeroEncoders() {
        TalonA.setSelectedSensorPosition(0);
        TalonB.setSelectedSensorPosition(0);
    }

    public void updateDash() {
        SmartDashboard.putNumber("Climb-Encoder Rev", returnRevs());
        SmartDashboard.putNumber("Climb-Encoder Veloctiy", getVel());
        // SmartDashboard.putNumber("ClimbDraw", getCurrentDraw());
        max = SmartDashboard.getNumber("ClimbMAX", -(Constants.CLIMB_MAX_POS));

        SmartDashboard.putBoolean("max", atMax);
    }

}
