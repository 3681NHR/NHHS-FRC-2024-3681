package frc.robot.interfaces;

import frc.robot.enums.MotionType;

public interface MotorInterface {

    public void set(MotionType type, Double value);

    public void stopMotor();

    public double get();

    public void setVelocity(double value);
}