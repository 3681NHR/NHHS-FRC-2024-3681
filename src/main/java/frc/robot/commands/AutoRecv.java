package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    int waitTimer = 0;
    int upTrig = 0;

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
        waitTimer = 0;
        upTrig = 0;
        cutoff = 0;
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_SWING.RECV_POSITION);
        
        if(m_launcherSwingSubsystem.isAtSelectedPos()){

            m_intakeSubsystem.setPosition(IntakeSwingState.UP);

            if(m_intakeSubsystem.isAtSelectedPos()){
                if(upTrig == 1){
                    waitTimer = cutoff;
                    upTrig = 2;
                } else {
                    if(upTrig == 0){
                        upTrig = 1;
                    }
                }
                if((!m_launcherSubsystem.isHolding() || m_intakeSubsystem.isHolding()) && cutoff-waitTimer >= 25){

                    m_launcherSubsystem.setRoller(RollerState.RECV);
                    m_intakeSubsystem.setIntake(IntakeState.REVERSE);
                } else {
                    m_intakeSubsystem.setIntake(IntakeState.IDLE);
                    m_launcherSubsystem.setRoller(RollerState.IDLE);
                }
            } else {
                m_intakeSubsystem.setIntake(IntakeState.IDLE);
                m_launcherSubsystem.setRoller(RollerState.IDLE);
            }
        }
        cutoff++;
    }

    @Override
    public boolean isFinished(){
        return (cutoff >= 500) || (!m_intakeSubsystem.isHolding() && m_launcherSubsystem.isHolding());
    }
}
