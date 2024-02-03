package frc.robot.wrappers;

import frc.robot.interfaces.MotorInterface;
import frc.robot.enums.IdleState;
import frc.robot.enums.MotionType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkWrapper implements MotorInterface{

    private int m_ID = 0;
    private CANSparkMax m_spm;
    private MotorType m_type;

    public SparkWrapper(int ID, MotorType type){
        m_ID = ID;
        m_type = type;

        m_spm = new CANSparkMax(m_ID, m_type);
    }

    public void set(MotionType type, Double value){
        if(type == MotionType.VELOCITY){
            m_spm.set(value);
        } else {
            System.out.println("a sparkwrapper was set using an unsuported motion type!");
        }
    }
    public void setVelocity(double value){
        m_spm.set(value);
    }

    public void stopMotor(){
        m_spm.stopMotor();
    }

    public double get(){
        return m_spm.get();
    }

    public void setIdleMode(IdleState mode){
        if(mode == IdleState.BRAKE)
            m_spm.setIdleMode(IdleMode.kBrake);
        if(mode == IdleState.BRAKE)
            m_spm.setIdleMode(IdleMode.kCoast);
    };
}
