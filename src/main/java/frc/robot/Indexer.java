package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Indexer {
    
    private WPI_TalonFX mIndexer;
    private DigitalInput bottomsensor, topsensor; 
    boolean indexerStat; //true is enabled, false is disabled  
    boolean ball; 
    //int ballcount;    

    public Indexer() {
        mIndexer = new WPI_TalonFX(Constants.INDEXERFALCON);
        mIndexer.configFactoryDefault();
        mIndexer.setInverted(InvertType.None);
        mIndexer.setNeutralMode(NeutralMode.Brake);

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

        if(Robot.operatorController.getAButton()) {
             //  ballcount = 0; 
             indexerStat = true; 
             ball = false; 
         } else if (Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD && indexerStat == true) {
             indexForward();
         }
         else if(Robot.operatorController.getBButton()) {
             indexForward();
      //   } else if (Robot.operatorController.getAButton()) {
        //     indexReverse();
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
 
         indexerStatus();
         updateSmartDashboard();
    }

        /**
     * Index balls until one reaches the loading position
     */
    public void indexerStatus() {
        if (ball == false && atBottom()) {    
            /*can't just be based on bottomsensor here because then it might stop 
            when bottom is triggered before top is when intaking 2nd ball */
                    ball = true; 
                    indexerStat = false; //stop?
        } 
        else if (atTop()) { 
            indexerStat = false;
        }
    }

      /**
     * Checks whether a ball is at the top sensor
     * 
     * @return True if there is a ball
     */
    public boolean atTop() {
        return !topsensor.get();
    }

    /**
     * Checks whether a ball is at the bottom sensor
     * 
     * @return True if there is a ball
     */
    public boolean atBottom() {
        return !bottomsensor.get();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Top Triggered", atTop());
        SmartDashboard.putBoolean("Bottom Triggered", !bottomsensor.get());
        SmartDashboard.putBoolean("Indexer Status", indexerStat);
        SmartDashboard.putBoolean("Ball?", ball);
       // SmartDashboard.putNumber("Ball Count", ballcount);
    }

    public void indexForward() {
        mIndexer.set(Constants.INDEXINGSPEED);
    }

    public void indexReverse() {
        mIndexer.set(-Constants.INDEXINGSPEED);
    }

    public void indexerStop() {
        mIndexer.set(0);
    }


}
