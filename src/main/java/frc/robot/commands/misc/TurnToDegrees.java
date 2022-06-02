package frc.robot.commands.misc;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnToDegrees extends CommandBase {

   
    private final Drive drivetrain;

    private double degrees;//#region;

    boolean turnSide;

    public TurnToDegrees(Drive subsystem, double turnAmt, boolean lr){
        drivetrain = subsystem;
        degrees = turnAmt;
        turnSide = lr;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        drivetrain.gyroTurn(degrees, turnSide);
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
