package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends CommandBase {

    private final Shooter shot;

    private double rpm, angle;
    private boolean ready;


    public AutoShooter(Shooter subsystem, double RPM, double Angle, boolean isReady){
        shot = subsystem;
        rpm = RPM;
        angle = Angle;
        ready = isReady;

        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        shot.autoShoot(ready, rpm, angle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interupted){

    }
    
}
