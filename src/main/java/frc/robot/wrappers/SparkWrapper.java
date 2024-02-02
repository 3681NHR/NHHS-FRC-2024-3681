package frc.robot.wrappers;

import frc.robot.interfaces.MotorInterface;
import frc.robot.enums.MotionType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    public void setIdleMode(IdleMode mode){
        m_spm.setIdleMode(mode);
    };
}
