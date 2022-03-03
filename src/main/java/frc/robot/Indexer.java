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
    DigitalInput bottomsensor, topsensor; // sensor = true means no block 
    boolean indexerStat; //true is enabled, false is disabled  
    boolean ball; 
    //int ballcount;

    public Indexer() {
        TalonA = new WPI_TalonFX(Constants.INDEXERFALCON);
        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));

        bottomsensor = new DigitalInput(Constants.BOTTOMSENSOR);
        topsensor = new DigitalInput(Constants.TOPSENSOR);

        indexerStat = true;
       // ballcount = 0;
        ball = false;

        SmartDashboard.putBoolean("Top Sensor Triggered", false);
        SmartDashboard.putBoolean("Bottom Sensor Triggered", false);
        SmartDashboard.putBoolean("Indexer Status", indexerStat);
        SmartDashboard.putBoolean("Ball?", ball);
      //  SmartDashboard.putNumber("Ball Count", ballcount); 
    }

    public void run() {
       
        if (Robot.operatorController.getXButtonPressed()) {
            indexerStat = !indexerStat;
        }

        if(Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            indexerUp(Constants.SHOOTINDEXINGSPEED);
          //  ballcount = 0; 
            indexerStat = true; 
            ball = false; 
        } else if (Robot.operatorController.getRightTriggerAxis() >= Constants.TRIGGER_THRESHOLD && indexerStat == true) {
            indexerUp(Constants.INDEXINGSPEED);
        }
        else if(Robot.operatorController.getBButton()) {
            indexerUp(Constants.INDEXINGSPEED);
        } else if (Robot.operatorController.getAButton()) {
            indexerUp(-Constants.INDEXINGSPEED);
        } else {
            indexerStop();
        }

       /* if (ballcount == 0 && !bottomsensor.get()) { 
            ballcount = 1;
            indexerStat = false; 
        } 
        else if (ballcount == 1 && bottomsensor.get()) { 
            ballcount = 2;
        }
        else if (!topsensor.get()) { 
            indexerStat = false;
        }
            */ 
    
        if (ball == false && !bottomsensor.get()) {    
    /*can't just be based on bottomsensor here because then it might stop 
    when bottom is triggered before top is when intaking 2nd ball */
            ball = true; 
            indexerStat = false; 
        } 
        else if (!topsensor.get()) { 
            indexerStat = false;
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
        SmartDashboard.putBoolean("Indexer Status", indexerStat);
        SmartDashboard.putBoolean("Ball?", ball);
       // SmartDashboard.putNumber("Ball Count", ballcount);
    }

}