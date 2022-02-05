package frc.robot;

import javax.swing.Action;
/*
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

/*
public class Climb {
    // Integrated Falcon 500 TalonFX
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    DoubleSolenoid Actuator;

    private boolean Hooked;
    public Climb() {
        Actuator = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[0],
                Constants.CLIMB_PNEUMATICS[1]);

        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and Add Factort Default Settings
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 10, 0.5));
        TalonB.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 55, 10, 0.5));

        FalconClimb = new MotorControllerGroup(TalonA, TalonB);

        // Encoder Config
        TalonA.configFactoryDefault();
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        Hooked = false;
    }

    // Mainloop
    public void run() {
        Telescop();
        PneumaticModes();
    }

    public void Telescop(){
        double lJoystickPos = Robot.operatorController.getLeftY();
        
        if (lJoystickPos > Constants.CLIMB_THRESHOLD){
            ExtendClimb();
        }
        else if (lJoystickPos < -Constants.CLIMB_THRESHOLD){
            RetractClimb();
        }
        /*
        else{
            FalconClimb.set(0);
        }*/
/*
    }

    public void PneumaticModes(){
        Contract();

        if (Robot.operatorController.getLeftBumper() && !Hooked){
            toggleStates();
        }
        if (Robot.operatorController.getRightBumper()){
            Release();
        }
    }

    public void toggleStates(){
        Actuator.toggle();
    }

    public void Extend() {
        Actuator.set(Value.kForward);
    }

    public void Contract() {
        Actuator.set(Value.kReverse);
    }

    public void Release() {
        Actuator.set(Value.kOff);
    }

    public void ExtendClimb() {
        FalconClimb.set(Constants.CLIMB_MOTOR_SPEED);
    }

    public void RetractClimb() {
        FalconClimb.set(-Constants.CLIMB_MOTOR_SPEED);
    }

*/
  //  public void ClampSpoolPos() {
        /*
            Get Encoder Pos
            Test Postition Control
            Shuffleboard

        */
/*
    }

    public void Shuffleboard(){
        //print values to shuffleboard
    }
    */

