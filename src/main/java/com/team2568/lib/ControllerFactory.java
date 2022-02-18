package com.team2568.lib;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2568.frc2022.Constants;

/**
 * Collection of static factory methods to create default controller objects
 * configured to the team's standard current limits.
 * 
 * To change the numbers, edit {@link com.team2568.frc2022.Constants}
 * 
 * @author Ryan Chaiyakul
 */
public class ControllerFactory {

    private static final SupplyCurrentLimitConfiguration mTalonSupplyCurr = new SupplyCurrentLimitConfiguration(true,
            Constants.kTalonSupplyCurrentLimit, Constants.kTalonSupplyTriggerCurrent,
            Constants.kTalonSupplyTriggerTime);
    private static final StatorCurrentLimitConfiguration mTalonStatorCurr = new StatorCurrentLimitConfiguration(true,
            Constants.kTalonStatorCurrentLimit, Constants.kTalonStatorTriggerCurrent,
            Constants.kTalonStatorTriggerTime);

    /**
     * Get a factory default {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX}
     * with generic current limits and brake mode. With the option of inverting it.
     * 
     * @param port
     * @param invert
     * @return
     */
    public static WPI_TalonFX getTalonFX(int port, boolean invert) {
        WPI_TalonFX ret = getTalonFX(port);
        ret.configFactoryDefault();

        ret.configSupplyCurrentLimit(mTalonSupplyCurr);
        ret.configStatorCurrentLimit(mTalonStatorCurr);

        ret.setNeutralMode(NeutralMode.Brake);

        ret.setInverted(invert);
        return ret;
    }

    /**
     * Get a factory default {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX}
     * with generic current limits and brake mode.
     * 
     * @param port
     * @return
     */
    public static WPI_TalonFX getTalonFX(int port) {
        return getTalonFX(port, false);
    }

    /**
     * Get a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX} follower
     * configured with team specified defaults. With the option of inverting it
     * (relative to the master direction).
     * 
     * @param master
     * @param port
     * @param invert
     * @return
     */
    public static WPI_TalonFX getTalonFXFollower(IMotorController master, int port, boolean invert) {
        WPI_TalonFX ret = getTalonFX(port);
        ret.follow(master);

        if (invert) {
            ret.setInverted(InvertType.OpposeMaster);
        } else {
            ret.setInverted(InvertType.FollowMaster);
        }
        return ret;
    }

    /**
     * Get a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX} follower
     * configured with team specified defaults.
     * 
     * @param master
     * @param port
     * @return
     */
    public static WPI_TalonFX getTalonFXFollower(IMotorController master, int port) {
        return getTalonFXFollower(master, port, false);
    }

    /**
     * Get a factory default {@link com.revrobotics.CANSparkMax} with generic
     * current limits and brake mode. With the option of inverting it.
     * 
     * @param port
     * @param invert
     * @return
     */
    public static CANSparkMax getSparkMAX(int port, boolean invert) {
        CANSparkMax ret = new CANSparkMax(port, MotorType.kBrushless);
        ret.restoreFactoryDefaults();

        ret.setSmartCurrentLimit(Constants.kSparkSmartCurrentLimit);
        ret.setSecondaryCurrentLimit(Constants.kSparkSecondaryCurrentLimit);

        ret.setIdleMode(IdleMode.kBrake);

        ret.setInverted(invert);
        return ret;
    }

    /**
     * Get a factory default {@link com.revrobotics.CANSparkMax} with generic
     * current limits and brake mode.
     * 
     * @param port
     * @return
     */
    public static CANSparkMax getSparkMAX(int port) {
        return getSparkMAX(port, false);
    }
}
