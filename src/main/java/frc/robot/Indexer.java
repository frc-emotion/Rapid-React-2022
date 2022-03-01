package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** @author Sachitt Arora
 * 
 * 
 * 
**/
public class Indexer {

    WPI_TalonFX TalonA;
    DigitalInput bottomsensor, topsensor;

    public Indexer() {
        bottomsensor = new DigitalInput(Constants.BOTTOMSENSOR);
        topsensor = new DigitalInput(Constants.TOPSENSOR);

        TalonA = new WPI_TalonFX(Constants.INDEXERFALCON);
        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));

        SmartDashboard.putBoolean("Top Sensor Triggered", false);
        SmartDashboard.putBoolean("Bottom Sensor Triggered", false);
    }

    public void run() {
        if(Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            indexerUp(Constants.SHOOTINDEXINGSPEED);
        } else if(Robot.operatorController.getBButton()) {
            indexerUp(Constants.INDEXINGSPEED);
        } else if (Robot.operatorController.getAButton()) {
            indexerUp(-Constants.INDEXINGSPEED);
        } else {
            indexerStop();
        }

        updateSmartDashboard();

    }

    public void indexerUp(double speed) {
        TalonA.set(speed);
    }

    public void indexerStop() {
        TalonA.set(0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Top Sensor Triggered", !topsensor.get());
        SmartDashboard.putBoolean("Bottom Sensor Triggered", !bottomsensor.get());

    }

}