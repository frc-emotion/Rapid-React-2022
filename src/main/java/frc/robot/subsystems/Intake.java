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
   public static DoubleSolenoid solenoidA, solenoidB, giga;

   public Intake() {

      intakeSpark = new CANSparkMax(Constants.INTAKE_PORT, MotorType.kBrushless);
      intakeSpark.setSmartCurrentLimit(Constants.NEO_MAX_CURRENT);
      intakeSpark.setSecondaryCurrentLimit(Constants.NEO_MAX_CURRENT);
      intakeSpark.setIdleMode(IdleMode.kBrake);




      solenoidA = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PNEUMATIC_INTAKE_PORTS[0],
            Constants.PNEUMATIC_INTAKE_PORTS[1]);
      solenoidA.set(Value.kForward);
   }

   public void run() {
      if (Robot.operatorController.getAButtonPressed()) {
         intakeToggle();
      }

      if (Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) { // && (Robot.indexer.ballcount
                                                                                          // < 2)
         intakeRoller();
      } else if (Robot.operatorController.getRightBumper()) {
         intakeRollerReverse();
      } else {
         intakeRollerOff();
      }
   }

   public void intakeToggle() {
      solenoidA.toggle();
   }

   public void intakeDown() {
      solenoidA.set(Value.kReverse);
   }

   public void intakeUp() {
      solenoidA.set(Value.kForward);
   }

   public void intakeRoller() {
      intakeSpark.set(-Constants.INTAKE_SPEED);
   }

   public void intakeRollerReverse() {
      intakeSpark.set(Constants.REVERSE_INTAKE_SPEED);
   }

   public void intakeRollerOff() {
      intakeSpark.set(0);
   }

}
