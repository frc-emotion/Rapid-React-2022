package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {
    
   private CANSparkMax intakeSpark; 
   private DoubleSolenoid solenoidA, solenoidB;

   public Intake() {
      
      intakeSpark = new CANSparkMax(Constants.INTAKE_PORT, MotorType.kBrushless); 
      intakeSpark.setSmartCurrentLimit(Constants.INTAKE_LIMIT);
      intakeSpark.setSecondaryCurrentLimit(Constants.INTAKE_LIMIT);
      intakeSpark.setIdleMode(IdleMode.kBrake);

      solenoidA = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_INTAKE_PORTS[0], Constants.PNEUMATIC_INTAKE_PORTS[1]); //check module type w/Bart
      solenoidB = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PNEUMATIC_INTAKE_PORTS[2], Constants.PNEUMATIC_INTAKE_PORTS[3]);
   }

   public void run(){
      if (Robot.operatorController.getYButtonPressed()) {
         intakeToggle();
      }

      if (Robot.operatorController.getRightTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
         intakeRoller();
      } else if (Robot.operatorController.getRightBumper()) {
         intakeRollerReverse();
      } else {
         intakeRollerOff();
      }

   }

   public void intakeToggle() {
      solenoidA.toggle();;
      solenoidB.toggle();
   }
   
   public void intakeDown() {
      solenoidA.set(Value.kForward);
      solenoidB.set(Value.kForward);
   }

   public void intakeUp() {
      solenoidA.set(Value.kReverse);
      solenoidB.set(Value.kReverse);
   }

   public void intakeRoller() {
      intakeSpark.set(Constants.INTAKE_SPEED);
   }   

   public void intakeRollerReverse() {
      intakeSpark.set(-Constants.INTAKE_SPEED);
   }

   public void intakeRollerOff() {
      intakeSpark.set(0);
   }

}
