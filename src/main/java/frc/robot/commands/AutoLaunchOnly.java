package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.LauncherState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class AutoLaunchOnly extends Command{
    
    LauncherSubsystem m_launcherSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;

    int ticks = 0;

    public AutoLaunchOnly(LauncherSwingSubsystem launcherSwing, LauncherSubsystem launcher){
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
        m_launcherSubsystem     .setSpeed(LauncherState.IDLE);
        m_launcherSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_launcherSubsystem.setSpeed(LauncherState.LAUNCHING);
        m_launcherSwingSubsystem.setAngle(Constants.LAUNCHER_SWING.LAUNCH_POSITION);

        if(m_launcherSubsystem.atspeed()){
            if(m_launcherSubsystem.isHolding()){
                m_launcherSubsystem.setRoller(RollerState.LAUNCH);
            } else {
                m_launcherSubsystem.setRoller(RollerState.IDLE);
            }
        }

        SmartDashboard.putNumber("auto ticks", ticks);
        ticks++;
    }

    @Override
    public boolean isFinished(){
        return ticks >= 450 || !m_launcherSubsystem.isHolding();//50ticks = 1sec
    }
}
