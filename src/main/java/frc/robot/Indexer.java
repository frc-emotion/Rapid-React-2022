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
    boolean indexerStat; //true means indexer is enabled, false is disabled  
    boolean firstBall; 
    int ballcount;    

    public Indexer() {
        mIndexer = new WPI_TalonFX(Constants.INDEXERFALCON);
        mIndexer.configFactoryDefault();
        mIndexer.setInverted(InvertType.None);
        mIndexer.setNeutralMode(NeutralMode.Brake);

        bottomsensor = new DigitalInput(Constants.BOTTOMSENSOR);
        topsensor = new DigitalInput(Constants.TOPSENSOR);

        indexerStat = true;
        firstBall= false;
        ballcount = 0;
    }

    public void run() {
        if (Robot.operatorController.getXButtonPressed()) { 
            indexerStat = !indexerStat; //changes indexer status 
        }

        if(Robot.operatorController.getAButton()) {
             indexerStat = true; 
             firstBall= false; 
             ballcount = 0; 
         } else if (Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD && indexerStat == true) {
             indexForward();
         } 
         else if(Robot.operatorController.getBButton()) {
             indexReverse();
         } 
         else {
             indexerStop();
         }
 
         indexerStatus();
         updateSmartDashboard();
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

 /**
     * Index balls until one reaches the loading position
     */
    public void indexerStatus() {
        if (firstBall == false && atBottom()) {    // if (ballcount == 0 && atBottom()) {  
                    firstBall = true; 
                    indexerStat = false; //stop?
                    ballcount = 1; 
        } 
        else if (atTop()) {  
            indexerStat = false;
        }
        
        if(atTop() && atBottom()) {
            ballcount = 2; 
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
        SmartDashboard.putBoolean("Bottom Triggered", atBottom());
        SmartDashboard.putBoolean("Indexer Status", indexerStat);
        SmartDashboard.putBoolean("First Ball?", firstBall);
        SmartDashboard.putNumber("Ball Count", ballcount);
    }

}
