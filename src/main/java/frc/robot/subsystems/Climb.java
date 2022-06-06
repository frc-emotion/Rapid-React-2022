package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.dashboard.TabManager;
import frc.robot.util.dashboard.TabManager.SubsystemTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * @author Karan Thakkar
 */

public class Climb extends SubsystemBase {

    public static final PowerDistribution PDP = new PowerDistribution();
    
    public static DoubleSolenoid ActuatorR, ActuatorL;
    public static WPI_TalonFX TalonA, TalonB;

    private boolean atMax;
    private double max;

    public NetworkTableEntry climbEncoderPosition;
    public NetworkTableEntry climbCurrentDraw;
    public NetworkTableEntry climbMaxPosition;
    public NetworkTableEntry climbAtMax;

    public Climb() {
        // Init Double Solenoids, Falcons
        ActuatorL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMB_PNEUMATICS[0],
                Constants.CLIMB_PNEUMATICS[1]);

        TalonA = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[0]);
        TalonB = new WPI_TalonFX(Constants.CLIMB_MOTORS_ID[1]);

        TalonA.setNeutralMode(NeutralMode.Brake);
        TalonB.setNeutralMode(NeutralMode.Brake);

        // Remove and add Factory Default Settings
        TalonA.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));
        TalonB.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.TALON_MAX_CURRENT, 10, 0.5));

        TalonB.follow(TalonA);

        // Encoder Config
        TalonA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        TalonB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        TalonA.config_kP(0, Constants.CLIMB_kP);
        TalonA.config_kI(0, Constants.CLIMB_kI);
        TalonA.config_kD(0, Constants.CLIMB_kD);

        // Output Encoder Values
        initShuffleboard();
        atMax = false;
        ActuatorL.set(Value.kReverse);
    }

    @Override
    public void periodic() {
        updateShuffleboard();
    }

    // Mainloop
    public void run() {
        double lJoystickPos = Robot.operatorController.getRightY();

        // Statements are Reversed, as the Joystick is reversed
        if (lJoystickPos < -Constants.JOYSTICK_THRESHOLD && !atMax) {
            TalonA.set(-Constants.CLIMB_MOTOR_SPEED);
        } else if (lJoystickPos > Constants.JOYSTICK_THRESHOLD) {
            TalonA.set(Constants.CLIMB_MOTOR_SPEED);
        } else if (Robot.operatorController.getStartButtonPressed()) {
            zeroEncoders();
        } else {
            TalonA.set(0);
        }
        if (Robot.operatorController.getYButtonPressed()) {
            ActuatorL.toggle();
        }
        Bounds();
    }

    /**
     * 
     * @return TalonFx Encoder Position (Encoder Units)
     */
    public double getPosition() {
        return TalonA.getSelectedSensorPosition();
    }

    public double getVel() {
        return TalonA.getSelectedSensorVelocity();
    }

    public void Bounds() {
        if (returnRevs() < -max) {
            atMax = true;
        } else {
            atMax = false;
        }
    }

    /** 
     * TalonFx Units Per Rev (NON QUADRATURE) = 2048 
     * 
     * */
    public double returnRevs() {
        return getPosition() / 2048;
    }

    private void zeroEncoders() {
        TalonA.setSelectedSensorPosition(0);
        TalonB.setSelectedSensorPosition(0);
    }

    private double getCurrentDraw(int channel){
        return PDP.getCurrent(channel);
    }

    public void initShuffleboard(){
        ShuffleboardTab climbData = TabManager.getInstance().accessTab(SubsystemTab.CLIMB);
        climbEncoderPosition = TabManager.getInstance().addWidget(climbData, BuiltInWidgets.kTextView, "Climb: Encoder Position", 0, new int[]{0,0}, new int[]{2,2});
        climbCurrentDraw = TabManager.getInstance().addWidget(climbData, BuiltInWidgets.kVoltageView, "Climb: Current Draw", 0, new int[]{2,0}, new int[]{2,2});
        climbMaxPosition = TabManager.getInstance().addWidget(climbData, BuiltInWidgets.kTextView, "Climb: Max Postion", -Constants.CLIMB_MAX_POS, new int[]{4,0}, new int[]{2,2});
        climbAtMax = TabManager.getInstance().addWidget(climbData, BuiltInWidgets.kBooleanBox, "Climb: At Max Position?", atMax, new int[]{6,0}, new int[]{2,2});

    }

    public void updateShuffleboard() {
        climbEncoderPosition.setDouble(returnRevs());
        climbCurrentDraw.setDouble(getCurrentDraw(14));
        climbMaxPosition.setBoolean(atMax);
        max = climbMaxPosition.getDouble(-(Constants.CLIMB_MAX_POS));
    }

}
