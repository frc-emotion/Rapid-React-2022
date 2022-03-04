package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * 
 * 
 * 
 * @author Karan Thakkar
 */

public class Climb {
    // Declare Integrated Falcon 500 TalonFX
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    Compressor Com;
    DoubleSolenoid ActuatorR, ActuatorL;


    AnalogPotentiometer pressure;
    double scale = 250, offset = -25;    //

    private double targetRev;

    // Once Hooked, release pressure, and zero encoderPos
    private boolean Hooked;
    private boolean forward;

    public Climb() {

        // Init Double Solenoids, Falcons
        ActuatorL = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10 , 5);
              
        ActuatorR = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 13 );

        pressure = new AnalogPotentiometer(0, scale, offset);
        Com = new Compressor(PneumaticsModuleType.REVPH);
        /** 
        ActuatorF = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[4],
                Constants.CLIMB_PNEUMATICS[5]);
        ActuatorG = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[6],
                Constants.CLIMB_PNEUMATICS[7]);
        */
        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and Add Factort Default Settings
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));
        TalonB.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));

        TalonB.follow(TalonA);

        // Encoder Config
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        TalonA.config_kP(0, Constants.CLIMB_kP);
        TalonA.config_kI(0, Constants.CLIMB_kI);
        TalonA.config_kD(0, Constants.CLIMB_kD);
        targetRev = Constants.CLIMB_TARGET_POS / 2048;
        // Output Encoder Values
        SmartDashboard.putNumber("Target Rev", Constants.CLIMB_TARGET_POS / 2048);
        

        //
        Hooked = false;
        
        ActuatorR.set(Value.kReverse);
        ActuatorL.set(Value.kReverse);
        //During Robot Init

        Com.enableAnalog(15, 120);
        ZeroEncoders();
    }

    // Mainloop
    public void run() {
        double lJoystickPos = Robot.operatorController.getLeftY();
        
        if(lJoystickPos > Constants.CLIMB_THRESHOLD && getPosition() < -Constants.CLIMB_MAX_POS){
            TalonA.set(0);
        }

        if(lJoystickPos > -Constants.CLIMB_THRESHOLD && getPosition() > Constants.CLIMB_MIN_POS){
            TalonA.set(0);
        }


        if (lJoystickPos > Constants.CLIMB_THRESHOLD) {
            TalonA.set(Constants.CLIMB_MOTOR_SPEED);
           // ExtendClimb(Constants.CLIMB_TARGET_POS);
        } else if (lJoystickPos < -Constants.CLIMB_THRESHOLD) {
            TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
            //RetractClimb(
           
        } else {
            TalonA.set(0);
        }
        
        if (Robot.operatorController.getLeftBumper() && !Hooked) {
            //toggleStates(ActuatorL);
            //toggleStates(ActuatorR);

            ActuatorL.toggle();
            ActuatorR.toggle();
            //ActuatorR.toggle();
        }

        if (Robot.operatorController.getAButton()){
            ZeroEncoders();
        }
     //   System.out.println(pressure.get());

        RunSmartDash();
      
    }


    // Methods for Motors/Pneumatics
    private void toggleStates(DoubleSolenoid Actuator) {
        Actuator.toggle();
    }

    private void Extend(DoubleSolenoid Actuator) {
        Actuator.set(Value.kReverse);
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
        // TalonA.set(Constants.CLIMB_MOTOR_SPEED);

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
    public double getVel() {
        return TalonA.getSelectedSensorVelocity();
    }

    public void PauseClimb(double targetPos) {
        if (getPosition() > targetPos && !Hooked) {
            stopClimb();
        } else if (Hooked && getPosition() < Constants.CLIMB_ZERO_POS) {
            ZeroEncoders();
        }
    }

    public double returnRevs(){
        //TalonFx Units Per Rev (NON QUADRATURE) = 2048 
        return getPosition() / 2048;


    }

    public void setRev(double revs){
        double setPoint = revs * 2048;

        //get pos once
        double currentPos = getPosition();

        TalonA.set(ControlMode.Position, currentPos + setPoint);
    }

    private void ZeroEncoders(){
        TalonA.setSelectedSensorPosition(0);
        TalonB.setSelectedSensorPosition(0);
    }

    public void RunSmartDash() {
        SmartDashboard.putNumber("Climb-Encoder Rev", getPosition());
        SmartDashboard.putNumber("Climb-Encoder Veloctiy", getVel());
        targetRev = SmartDashboard.getNumber("Target Rev", (Constants.CLIMB_TARGET_POS / 2048) * 2048);
    }

}
