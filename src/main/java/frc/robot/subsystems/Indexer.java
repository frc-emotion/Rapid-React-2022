package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.dashboard.TabManager;
import frc.robot.util.dashboard.TabManager.SubsystemTab;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Indexer extends SubsystemBase {

    private CANSparkMax mIndexer;
    private DigitalInput bottomsensor, topsensor, intakesensorR, intakesensorL;
    boolean indexerStat; // true means indexer is enabled, false is disabled
    boolean firstBall;
    boolean indexingFirstBall;
    boolean indexingSecondBall;
    int ballcount;

    NetworkTableEntry topTriggered;
    NetworkTableEntry bottomTriggered;
    NetworkTableEntry intakeSensor;
    NetworkTableEntry ballCount;

    public Indexer() {
        mIndexer = new CANSparkMax(Constants.INDEXERFALCON, MotorType.kBrushless);
        // mIndexer.configFactoryDefault();
        // mIndexer.setInverted(InvertType.None);
        // mIndexer.setNeutralMode(NeutralMode.Brake);
        mIndexer.setIdleMode(IdleMode.kBrake);
        mIndexer.setSmartCurrentLimit(45);
        mIndexer.setSecondaryCurrentLimit(45);

        intakesensorR = new DigitalInput(Constants.INTAKESENSORR);
        intakesensorL = new DigitalInput(Constants.INTAKESENSORL);
        bottomsensor = new DigitalInput(Constants.BOTTOMSENSOR);
        topsensor = new DigitalInput(Constants.TOPSENSOR);

        firstBall = false;
        indexingFirstBall = false;
        indexingSecondBall = false;
        ballcount = 0;

        initShuffleboard();
    }

    public void run() {
        if (Robot.operatorController.getLeftBumper()) {
            indexForward(Constants.INDEXINGSPEED);
        } else if (Robot.operatorController.getXButton()) {
            indexShoot(Constants.SHOOTINDEXINGSPEED);
        } else if (Robot.operatorController.getBButton()) {
            indexReverse(Constants.INDEXINGSPEED);
        } else if (atTop()) {
            indexStop();
        } else {
            indexStop();
        }

    }

    @Override
    public void periodic() {
        updateShuffleboard();
    }

    public void indexForward(double speed) {
        mIndexer.set(speed);
    }

    public void indexReverse(double speed) {
        mIndexer.set(-speed);
    }

    public void indexStop() {
        mIndexer.set(0);
    }

    public void indexShoot(double speed) {
        indexForward(speed);
        firstBall = false;
    }

    /**
     * Index balls until one reaches the loading position
     */
    public void checkBallCount() {
        if (atTop()) {
            ballcount = 2;
        } else if (firstBall == true) {
            ballcount = 1;
        }
    }

    public void indexingFirstBall() {
        if (atBottom()) {
            firstBall = true;
            indexingFirstBall = false;
        } else {
            indexForward(Constants.INDEXINGSPEED);
        }
    }

    public void indexingSecondBall() {
        if (atTop()) {
            indexingSecondBall = false;
        } else {
            indexForward(Constants.INDEXINGSPEED);
        }
    }

    public void autoIndex(boolean x) {
        x = false;
        if (!atTop()) {
            indexForward(Constants.INDEXINGSPEED);
        }
        if (atTop()) {
            x = true;
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

    /**
     * Checks whether a ball is at the intake sensor
     * 
     * @return True if there is a ball
     */
    public boolean atIntake() {
        return (!intakesensorR.get() || !intakesensorL.get());
    }

    /**
     * @param shooting
     */
    public void runAutoIndexer(boolean shooting) {
        if (shooting) {

        } else if (atTop()) {
            indexStop();

        } else if (atIntake() && firstBall == false) {
            indexingFirstBall();

        } else if (atIntake() && firstBall == true) {
            indexForward(Constants.INDEXINGSPEED); // will auto stop when it reaches top

        } else {
            indexStop();
        }
    }

    private void initShuffleboard(){
        ShuffleboardTab indexerData = TabManager.getInstance().accessTab(SubsystemTab.INDEXER);
    }

    private void updateShuffleboard() {
    /*
        SmartDashboard.putBoolean("Top Triggered", atTop());
        SmartDashboard.putBoolean("Bottom Triggered", atBottom());
        SmartDashboard.putBoolean("Intake Triggered", atIntake());
        SmartDashboard.putBoolean("First Ball?", firstBall);
        SmartDashboard.putNumber("Ball Count", ballcount); 
        
    */
    }

}
