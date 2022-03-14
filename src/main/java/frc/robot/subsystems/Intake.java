package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    
   private CANSparkMax intakeSpark; 
   private DoubleSolenoid solenoidA, solenoidB;

   public Intake() {
      
      intakeSpark = new CANSparkMax(Constants.INTAKE_PORT, MotorType.kBrushless); 
      intakeSpark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
      intakeSpark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
      intakeSpark.setIdleMode(IdleMode.kBrake);

      solenoidA = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_INTAKE_PORTS[0], Constants.PNEUMATIC_INTAKE_PORTS[1]); 
      solenoidA.set(Value.kForward);
      solenoidB = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_INTAKE_PORTS[2], Constants.PNEUMATIC_INTAKE_PORTS[3]);
      solenoidB.set(Value.kForward);
   }

   public void run(){
      if (Robot.operatorController.getYButtonPressed()) {
         intakeToggle();
      } 

      if (Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) { //&& (Robot.indexer.ballcount < 2)
         intakeRoller();
      } else if (Robot.operatorController.getBButton()) {
         //intakeRollerReverse();
      } else {
         intakeRollerOff();
      }



   }

   public void intakeToggle() {
      solenoidA.toggle();
      solenoidB.toggle();
   }  
   
   public void intakeDown() {
      solenoidA.set(Value.kReverse);
      solenoidB.set(Value.kReverse);
   }

   public void intakeUp() {
      solenoidA.set(Value.kForward);
      solenoidB.set(Value.kForward);
   }

   public void intakeRoller() {
      intakeSpark.set(-Constants.INTAKE_SPEED);
   }   

   public void intakeRollerReverse() {
      intakeSpark.set(Constants.INTAKE_SPEED);
   }

   public void intakeRollerOff() {
      intakeSpark.set(0);
   }

}
