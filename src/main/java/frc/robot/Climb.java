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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


/**
 * 
 * 
 * 
 * @author Karan Thakkar
 */

public class Climb {
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    DoubleSolenoid ActuatorR, ActuatorL;

    private boolean Hooked; 
    private boolean atMax;
    private boolean atMin;

    private PowerDistribution PDH;

    private double targetRev;

    public Climb() {

        PDH = new PowerDistribution();

        // Init Double Solenoids, Falcons
        ActuatorL = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[0], Constants.CLIMB_PNEUMATICS[1]);
        ActuatorR = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.CLIMB_PNEUMATICS[2], Constants.CLIMB_PNEUMATICS[3]);

        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and Add Factory Default Settings
        TalonA.configFactoryDefault();
        TalonB.configFactoryDefault();
        //TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));
        //TalonB.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));

        TalonB.follow(TalonA);

        // Encoder Config
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        TalonA.config_kP(0, Constants.CLIMB_kP);
        TalonA.config_kI(0, Constants.CLIMB_kI);
        TalonA.config_kD(0, Constants.CLIMB_kD);
      
        // Output Encoder Values
        SmartDashboard.putNumber("MAX_REV", Constants.CLIMB_TARGET_MAX_POS);
        Hooked = false;
        atMax = false;
        atMin = false;

        ActuatorR.set(Value.kReverse);
        ActuatorL.set(Value.kReverse);
        //During Robot Init

    }

    // Mainloop
    public void run() {
        double lJoystickPos = Robot.operatorController.getLeftY();
        if (!atMax || !atMin){
            if (lJoystickPos > Constants.JOYSTICK_THRESHOLD ) {
                TalonA.set(Constants.CLIMB_MOTOR_SPEED);
               // ExtendClimb(Constants.CLIMB_TARGET_POS);
            } else if (lJoystickPos < -Constants.JOYSTICK_THRESHOLD) {
                TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
                //RetractClimb(Constants.CLIMB_TARGET_POS);
            } else {
                TalonA.set(0);
            }
        }

        
        if (Robot.operatorController.getLeftBumperPressed() && !Hooked) {

            ActuatorL.toggle();
            ActuatorR.toggle();
        }

        Bounds();
        RunSmartDash();
      
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

    public void Bounds(){
        if (returnRevs() > Constants.CLIMB_MAX_POS){
            stopClimb();
            atMax = true;
        } else if (returnRevs() < Constants.CLIMB_MIN_POS){
            stopClimb();
            atMin = true;
        } else if (getCurrentDraw() > Constants.CLIMB_DRAW_THRESHOLD){
           // ZeroEncoders();
        }
    }

    public double returnRevs(){
        //TalonFx Units Per Rev (NON QUADRATURE) = 2048 
        return getPosition() / 2048;


    }

    public void setRev(double revs){
        double setPoint = revs * 2048;

        TalonA.set(ControlMode.Position, setPoint);
    }

    private void ZeroEncoders(){
        TalonA.setSelectedSensorPosition(0);
        TalonB.setSelectedSensorPosition(0);
    }
    
    public double getCurrentDraw(){
        return PDH.getCurrent(Constants.CLIMB_POWER_CHANNEL);
        //Create object of PDH
    }

    public void RunSmartDash() {
        SmartDashboard.putNumber("Climb-Encoder Rev", returnRevs());
        SmartDashboard.putNumber("Climb-Encoder Veloctiy", getVel());
        targetRev = SmartDashboard.getNumber("Target Rev", (Constants.CLIMB_MAX_POS / 2048) * 2048);
    }

}
