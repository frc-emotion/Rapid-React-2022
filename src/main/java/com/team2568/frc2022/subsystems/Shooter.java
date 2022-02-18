package com.team2568.frc2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2568.frc2022.Constants;
import com.team2568.frc2022.Registers;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter extends Subsystem {
    private static Shooter mSubsystem = new Shooter();
    private CANSparkMax mHood;
    private WPI_TalonFX mL, mR; // 2 Motors spin the shooting wheel

    public static Shooter getInstance() {
        return mSubsystem;
    }

    private Shooter() {
        mHood = new CANSparkMax(Constants.kShooterHoodPort, MotorType.kBrushless);
        mL = new WPI_TalonFX(Constants.kShooterLeftPort);
        mR = new WPI_TalonFX(Constants.kShooterRightPort);
        mR.follow(mL);
    }

    /**
     * Read from results from registers and set motors etc.
     */
    public void setOutputs() {
        switch (Registers.kShooterState.get()) {
            case OFF:
                stopAll();
                break;
            case ADJUSTING:
                stopShooter();
                break;
            case CHARGING:
                break;
            case FIRING:
                break;
            case CALIBRATING:
                break;
        }
    }

    /**
     * Stop all motors
     */
    private void stopAll() {
        mHood.stopMotor();
        mL.stopMotor();
    }

    /**
     * Stop only the shooter motors
     */
    private void stopShooter() {
        mL.stopMotor();
    }

    /**
     * Write to status registers
     */
    public void writeStatus() {

    }

    /**
     * Output permanent statements
     */
    public void writeDashboard() {

    }

    /**
     * Debugging dashboard statements
     */
    public void outputTelemetry() {

    }
}
