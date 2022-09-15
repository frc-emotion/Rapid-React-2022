package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ClimbTeleop extends CommandBase{

    private final Climb climb;
    public ClimbTeleop(Climb subsystem){
        climb = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
           // climb.run();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
