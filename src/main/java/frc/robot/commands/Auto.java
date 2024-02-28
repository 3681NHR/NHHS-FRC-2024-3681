package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;
import frc.robot.enums.LauncherState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class Auto extends Command{
    
    LauncherSubsystem m_launcherSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;

    int ticks = 0;

    public Auto(LauncherSwingSubsystem launcherSwing, LauncherSubsystem launcher){
        m_launcherSubsystem      = launcher;
        m_launcherSwingSubsystem = launcherSwing;


        addRequirements(launcher);
        addRequirements(launcherSwing);

    }
    @Override
    public void initialize(){
        ticks = 0;
    }

    @Override
    public void end(boolean e){
        m_launcherSubsystem     .setSpeedCommand(LauncherState.IDLE);
        m_launcherSwingSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_launcherSubsystem.setSpeed(LauncherState.LAUNCHING);
        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_LAUNCH_POSITION);

        if(ticks >= 5){
            m_launcherSwingSubsystem.setRoller(RollerState.RECV);
        }

        ticks++;
    }

    @Override
    public boolean isFinished(){
        return ticks >= 50;
    }
}
