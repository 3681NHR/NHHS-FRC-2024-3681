package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.IntakeState;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;

public class AutoRecv extends Command{
    
    IntakeSubsystem m_intakeSubsystem;
    LauncherSwingSubsystem m_launcherSwingSubsystem;

    int ticks = 0;

    public AutoRecv(IntakeSubsystem intake, LauncherSwingSubsystem launcherSwing){
        m_intakeSubsystem        = intake;
        m_launcherSwingSubsystem = launcherSwing;


        addRequirements(intake);
        addRequirements(launcherSwing);

    }
    @Override
    public void initialize(){
        ticks = 0;
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_launcherSwingSubsystem.setRoller(RollerState.IDLE);
    }

    @Override
    public void execute(){

        m_intakeSubsystem.setPosition(Constants.INTAKE_SWING_UP_POSITION);
        m_launcherSwingSubsystem.setPosition(Constants.LAUNCHER_RECV_POSITION);


        if(m_launcherSwingSubsystem.isAtSelectedPos() && m_intakeSubsystem.isAtSelectedPos()){
            m_launcherSwingSubsystem.setRoller(RollerState.RECV);
            m_intakeSubsystem.setIntake(IntakeState.REVERSE);
            ticks++;
        }
        
    }

    @Override
    public boolean isFinished(){
        return ticks >= 50;
    }
}
