package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterTeleop extends CommandBase{

    private final Shooter shooter;


    public ShooterTeleop(Shooter subsystem){
        shooter = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        shooter.run();
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
