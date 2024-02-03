package frc.robot.interfaces;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.enums.IdleState;
import frc.robot.enums.MotionType;

public interface MotorInterface {

    public void set(MotionType type, Double value);

    public void stopMotor();

    public double get();

    public void setVelocity(double value);

    public void setIdleMode(IdleState mode);
}