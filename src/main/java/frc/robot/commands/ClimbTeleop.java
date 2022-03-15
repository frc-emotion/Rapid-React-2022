package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;

public class ClimbTeleop extends CommandBase{

    private final Climb climb;

    private double speed;

    public ClimbTeleop(Climb subsystem){
        climb = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
            climb.run();
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
