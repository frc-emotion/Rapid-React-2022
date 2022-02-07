package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climb {
    // Declare Integrated Falcon 500 TalonFX
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    DoubleSolenoid ActuatorL;
    DoubleSolenoid ActuatorR;

    // Once Hooked, release pressure, and zero encoderPos
    private boolean Hooked;

    public Climb() {

        // Init Double Solenoids, Falcons
        ActuatorL = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[0],
                Constants.CLIMB_PNEUMATICS[1]);
        ActuatorR = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[2],
                Constants.CLIMB_PNEUMATICS[3]);

        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and Add Factort Default Settings
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 10, 0.5));
        TalonB.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 10, 0.5));

        TalonB.follow(TalonA);

        // Encoder Config
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        TalonA.config_kP(0, Constants.CLIMB_kP);
        TalonA.config_kI(0, Constants.CLIMB_kI);
        TalonA.config_kD(0, Constants.CLIMB_kD);

        //Output Encoder Values
        SmartDashboard.putNumber("Climb-Encoder Rev", 0);

        //
        Hooked = false;
        
        //During Robot Init
        Contract();
    }

    // Mainloop
    public void run() {
        Telescop();
        PneumaticModes();
        RunShuffleboard();
    }

    // Climb Loop
    public void Telescop() {
        double lJoystickPos = Robot.operatorController.getLeftY();

        if (lJoystickPos > Constants.CLIMB_THRESHOLD) {
            ExtendClimb(Constants.CLIMB_TARGET_POS);
        } else if (lJoystickPos < -Constants.CLIMB_THRESHOLD) {
            RetractClimb(Constants.CLIMB_TARGET_POS);
        } else {
            PauseClimb(Constants.CLIMB_TARGET_POS);
        }
    }

    // Solenoid Loop
    public void PneumaticModes() {
        Contract(ActuatorL);
        Contract(ActuatorR);

        if (Robot.operatorController.getLeftBumper() && !Hooked) {
            toggleStates(ActuatorL);
            toggleStates(ActuatorR);
        }

        // for testing purposes
        else if (Robot.operatorController.getRightBumper()) {
            Release(ActuatorR);
            Release(ActuatorL);
        }
    }

    // Methods for Motors/Pneumatics
    private void toggleStates(DoubleSolenoid Actuator) {
        Actuator.toggle();
    }

    private void Extend(DoubleSolenoid Actuator) {
        Actuator.set(Value.kForward);
    }

    public void Contract(DoubleSolenoid Actuator) {
        Actuator.set(Value.kReverse);
    }

    private void Release(DoubleSolenoid Actuator) {
        Actuator.set(Value.kOff);
    }

    private void ExtendClimb(double target) {
        TalonA.set(ControlMode.Position, target);
        PauseClimb(target);
        //TalonA.set(Constants.CLIMB_MOTOR_SPEED);

        
    }

    private void RetractClimb(double target) {
        TalonA.set(ControlMode.Position, -target);
        PauseClimb(target);
       // TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
    }

    private void stopClimb() {
        TalonA.set(0);
    }

    // Return TalonFX Position
    public double getPosition() {
        return TalonA.getSelectedSensorPosition();
    }

    public void PauseClimb(double targetPos) {
        if (getPosition() > targetPos && !Hooked) {
            stopClimb();
        }
        else if(Hooked && getPosition() < Constants.CLIMB_ZERO_POS){
            ZeroEncoders();
        }
    }

    private void ZeroEncoders(){
        TalonA.setSelectedSensorPosition(0);
        TalonB.setSelectedSensorPosition(0);
    }

    public void RunShuffleboard() {
        SmartDashboard.putNumber("Climb-Encoder Rev", getPosition());
    }

}