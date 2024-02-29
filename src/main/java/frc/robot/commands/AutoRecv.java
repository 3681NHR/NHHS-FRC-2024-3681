package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    int ticks = 0;
    int cutoff = 0;

    public AutoRecv(IntakeSubsystem intake, LauncherSwingSubsystem launcherSwing){
        m_intakeSubsystem        = intake;
        m_launcherSwingSubsystem = launcherSwing;


        addRequirements(intake);
        addRequirements(launcherSwing);

    }
    @Override
    public void initialize(){
        ticks = 0;
        SmartDashboard.putBoolean("autorecv running", true);
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSwingSubsystem.setRoller(RollerState.IDLE);

        SmartDashboard.putBoolean("autorecv running", false);
    }

    @Override
    public void execute(){

        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_RECV_POSITION);
        
        if(m_launcherSwingSubsystem.isAtSelectedPos()){

            m_intakeSubsystem.setPosition(IntakeSwingState.UP);

            if(m_intakeSubsystem.isAtSelectedPos()){
                m_launcherSwingSubsystem.setRoller(RollerState.RECV);
                m_intakeSubsystem.setIntake(IntakeState.REVERSE);
            
                ticks++;
            }
        }

        cutoff++;
    }

    @Override
    public boolean isFinished(){
        return (ticks >= 125) || (cutoff >= 1000);
    }
}
