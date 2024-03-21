package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class AutoRecv extends Command{
    
    IntakeSubsystem m_intakeSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;

    int cutoff = 0;

    public AutoRecv(IntakeSubsystem intake, LauncherSwingSubsystem launcherSwing){
        m_intakeSubsystem        = intake;
        m_launcherSwingSubsystem = launcherSwing;


        addRequirements(intake);
        addRequirements(launcherSwing);

    }
    @Override
    public void initialize(){
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSwingSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_LAUNCH_POSITION);
        
        if(m_launcherSwingSubsystem.isAtSelectedPos()){

            m_intakeSubsystem.setPosition(IntakeSwingState.UP);

            if(m_intakeSubsystem.isAtSelectedPos()){
                
                if(!m_launcherSwingSubsystem.isHolding() || m_intakeSubsystem.isHolding()){
                    m_launcherSwingSubsystem.setRoller(RollerState.RECV);
                    m_intakeSubsystem.setIntake(IntakeState.REVERSE);
                } else {
                    m_intakeSubsystem.setIntake(IntakeState.IDLE);
                    m_launcherSwingSubsystem.setRoller(RollerState.IDLE);
                }
            }
        }
        cutoff++;
    }

    @Override
    public boolean isFinished(){
        return (cutoff >= 1000) || (!m_intakeSubsystem.isHolding() && m_launcherSwingSubsystem.isHolding());
    }
}
