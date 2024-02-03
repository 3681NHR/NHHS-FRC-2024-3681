package frc.robot.wrappers;

import frc.robot.interfaces.MotorInterface;
import frc.robot.enums.IdleState;
import frc.robot.enums.MotionType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class VictorWrapper implements MotorInterface{

    private int m_ID = 0;
    private VictorSPX m_spm;

    public VictorWrapper(int ID){
        m_ID = ID;

        m_spm = new VictorSPX(m_ID);
    }

    public void set(MotionType type, Double value){
        if(type == MotionType.VELOCITY){
            m_spm.set(ControlMode.PercentOutput, value);
        } else {
            System.out.println("a victorwrapper was set using an unsuported motion type!");
        }
    }
    public void setVelocity(double value){
        m_spm.set(ControlMode.Velocity, value);
    }

    public void stopMotor(){
        m_spm.set(ControlMode.PercentOutput, 0);
    }

    public double get(){
        return m_spm.getMotorOutputPercent();
    }

    public void setIdleMode(IdleState mode){
        if(mode == IdleState.BRAKE)
            m_spm.setNeutralMode(NeutralMode.Brake);
        if(mode == IdleState.COAST)
            m_spm.setNeutralMode(NeutralMode.Coast);
    };
}
