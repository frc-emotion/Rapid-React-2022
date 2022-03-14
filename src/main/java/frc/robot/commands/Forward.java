package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Forward extends CommandBase{

    private final Drive drivetrain;
    private double speed;


    public Forward(Drive subsystem, double spd){
        drivetrain = subsystem;
        speed = spd;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        drivetrain.autoforward(speed);
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
