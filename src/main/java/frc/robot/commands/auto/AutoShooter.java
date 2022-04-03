package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
