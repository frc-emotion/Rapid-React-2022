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


public class Indexer {

    WPI_TalonFX TalonA;
    DigitalInput bottomsensor, topsensor; // sensor = true means no block 
    boolean containsFirstBall; //used to track whether there is a preexisting first ball
    boolean active;
    boolean released;

    public Indexer() {
        TalonA = new WPI_TalonFX(Constants.INDEXERFALCON);
        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonA.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));

        bottomsensor = new DigitalInput(Constants.BOTTOMSENSOR);
        topsensor = new DigitalInput(Constants.TOPSENSOR);
        containsFirstBall = false;
        active = false;
        released = false;

        SmartDashboard.putBoolean("Top Sensor Triggered", false);
        SmartDashboard.putBoolean("Bottom Sensor Triggered", false);
        SmartDashboard.putBoolean("First Ball", false);
    }

    public void run() {
        if(Robot.operatorController.getLeftTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            indexerUp(Constants.SHOOTINDEXINGSPEED);
        } else if(Robot.operatorController.getBButton()) {
            indexerUp(Constants.INDEXINGSPEED);
        } else if (Robot.operatorController.getAButton()) {
            indexerUp(-Constants.INDEXINGSPEED);
        } else if (!topsensor.get()) {
            indexerStop();
        } else if(Robot.operatorController.getXButtonPressed()) {
            released = true; //only for rare situations in which operator needs to change this again, otherwise indexing should run in conjunction with intake mechanism and just the intaking button
        } else if (Robot.operatorController.getRightTriggerAxis() >= Constants.TRIGGER_THRESHOLD) {
            active = true;
            if(!bottomsensor.get()) {
                containsFirstBall = true;
            }

            if(containsFirstBall) {
                if(topsensor.get() && released) {//run while unobstructed
                    indexerUp(Constants.INDEXINGSPEED);
                } else {
                    indexerStop();
                }

            } else if (!containsFirstBall) {
                if(bottomsensor.get()) {//redundant
                    indexerUp(Constants.INDEXINGSPEED);
                } else {
                    indexerStop();
                    containsFirstBall = true;
                }
            }
        } else if (active == true && Robot.operatorController.getRightTriggerAxis() < Constants.TRIGGER_THRESHOLD) {
            released = !released;
            active = false;
        } else {
            indexerStop();
        }

        System.out.println("active: " + active + " released" + released);

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
        SmartDashboard.putBoolean("First Ball", containsFirstBall);
    }

}