package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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

public class Climb extends SubsystemBase{
    MotorControllerGroup FalconClimb;
    WPI_TalonFX TalonA, TalonB;
    DoubleSolenoid ActuatorR, ActuatorL;

    private boolean Hooked; 
    private boolean atMax;
    private boolean atMin;

    private PowerDistribution PDH;

    public double max;
    public double min;
    public double maxCurrentDraw;

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
        SmartDashboard.putNumber("ClimbMAX", (Constants.CLIMB_MAX_POS));
        SmartDashboard.putNumber("ClimbMIN", 0);


        Hooked = false;
        atMax = false;
        atMin = false;

        ActuatorR.set(Value.kReverse);
        ActuatorL.set(Value.kReverse);
        //During Robot Init

    }

    @Override
    public void periodic() {
        RunSmartDash();
    }

    // Mainloop
    public void run() {
        double lJoystickPos = Robot.operatorController.getLeftY();

        //Statements are Reversed, as the Joystick is reversed
        if (lJoystickPos > Constants.JOYSTICK_THRESHOLD && !atMin) {
            TalonA.set(Constants.CLIMB_MOTOR_SPEED);
        } else if (lJoystickPos < -Constants.JOYSTICK_THRESHOLD && !atMax) {
            TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
        } else if (Robot.operatorController.getStartButtonPressed()) {
            ZeroEncoders();
        }
        else {
            TalonA.set(0);


        }
        if (Robot.operatorController.getLeftBumperPressed() && !Hooked) {
            ActuatorL.toggle();
            ActuatorR.toggle();
        }

        Bounds();
        
      
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
        // Talon.set(Constants.CLIMB_MOTOR_SPEED);

    }

    private void RetractClimb(double target) {
        TalonA.set(ControlMode.Position, -target);
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

    public void Bounds(){
        if (returnRevs() > max){
            atMax = true;
        } 
        else {
            atMax = false;
        }

        //lower bounds
        if (returnRevs() < min){
            atMin = true;
        }
        else {
            atMin = false;
        }

        //Disabled for testing, define maxCurrentDraw
        /*
        if (getCurrentDraw() > maxCurrentDraw){
            ZeroEncoders();
        }
        */
    }

    public double returnRevs(){
        //TalonFx Units Per Rev (NON QUADRATURE) = 2048 
        return getPosition() / 2048;


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
        SmartDashboard.putNumber("ClimbDraw", getCurrentDraw());
        max = SmartDashboard.getNumber("ClimbMAX", (Constants.CLIMB_MAX_POS));
        min = SmartDashboard.getNumber("ClimbMIN", 0);
    }

}
