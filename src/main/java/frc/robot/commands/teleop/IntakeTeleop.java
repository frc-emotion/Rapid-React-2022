package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeTeleop extends CommandBase{

    private final Intake intake;


    public IntakeTeleop(Intake subsystem){
        intake = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        intake.run();
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
