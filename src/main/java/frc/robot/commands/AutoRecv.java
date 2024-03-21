package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class AutoRecv extends Command{
    
    IntakeSubsystem m_intakeSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;
    LauncherSubsystem m_launcherSubsystem;

    int cutoff = 0;

    public AutoRecv(IntakeSubsystem intake, LauncherSwingSubsystem launcherSwing, LauncherSubsystem  launcher){
        m_intakeSubsystem        = intake;
        m_launcherSwingSubsystem = launcherSwing;
        m_launcherSubsystem      = launcher;

        addRequirements(intake);
        addRequirements(launcherSwing);
        addRequirements(launcher);
    }
    @Override
    public void initialize(){
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_LAUNCH_POSITION);
        
        if(m_launcherSwingSubsystem.isAtSelectedPos()){

            m_intakeSubsystem.setPosition(IntakeSwingState.UP);

            if(m_intakeSubsystem.isAtSelectedPos()){
                
                if(!m_launcherSubsystem.isHolding() || m_intakeSubsystem.isHolding()){
                    m_launcherSubsystem.setRoller(RollerState.RECV);
                    m_intakeSubsystem.setIntake(IntakeState.REVERSE);
                } else {
                    m_intakeSubsystem.setIntake(IntakeState.IDLE);
                    m_launcherSubsystem.setRoller(RollerState.IDLE);
                }
            }
        }
        cutoff++;
    }

    @Override
    public boolean isFinished(){
        return (cutoff >= 1000) || (!m_intakeSubsystem.isHolding() && m_launcherSubsystem.isHolding());
    }
}
