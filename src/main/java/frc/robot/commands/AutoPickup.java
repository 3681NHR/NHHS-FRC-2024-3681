package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoPickup extends Command{
    
    IntakeSubsystem m_intakeSubsystem;

    public AutoPickup(IntakeSubsystem intake){
        m_intakeSubsystem = intake;

        addRequirements(intake);
    }
    @Override
    public void initialize(){
        m_intakeSubsystem.setPosition(IntakeSwingState.DOWN);
        m_intakeSubsystem.setIntake(IntakeState.INTAKE);
    }

    @Override
    public void end(boolean e){
        m_intakeSubsystem.setIntake(IntakeState.IDLE);
        m_intakeSubsystem.setPosition(IntakeSwingState.UP);
    }

    @Override
    public void execute(){
        if(!m_intakeSubsystem.isHolding()){
            m_intakeSubsystem.setIntake(IntakeState.INTAKE);
        } else {
            m_intakeSubsystem.setIntake(IntakeState.IDLE);
        }
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.isHolding();
    }
}
