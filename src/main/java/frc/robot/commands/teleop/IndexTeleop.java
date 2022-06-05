package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexTeleop extends CommandBase{

    private final Indexer indexer;


    public IndexTeleop(Indexer subsystem){
        indexer = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        indexer.run();
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
