// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.RollerState;

public class LauncherSwingSubsystem extends SubsystemBase {

  private CANSparkMax swingMotor  = new CANSparkMax(Constants.LAUNCHER_SWING_MOTOR_ID, MotorType.kBrushless);
  private VictorSPX   roller      = new VictorSPX(Constants.LAUNCHER_ROLLER_MOTOR_ID);

  private DutyCycleEncoder swingEncoder = new DutyCycleEncoder(Constants.LAUNCHER_SWING_ENCODER_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);

  private double selectedPosition = swingEncoder.getDistance();

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);

  private RollerState rollerState = RollerState.IDLE;
  public LauncherSwingSubsystem() 
  {
    this.swingMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setRoller(RollerState state){
    rollerState = state;
  }
  public Command setRollerCommand(RollerState state){
    return runOnce(() -> {
      rollerState = state;
    });
  }

  public double getPosition(boolean selectedPos){
    if(selectedPos){
      return selectedPosition;
    } else{
    return swingEncoder.getDistance();
    }
  }

  public void setPosition(double pos){
    selectedPosition = pos;
  }

  public Command setPositionCommand(double pos){
    return runOnce(() -> {
    selectedPosition = pos;
    });
  }

  public boolean isAtSelectedPos(){
    if(Math.abs(selectedPosition - swingEncoder.getDistance()) <= Constants.LAUNCHER_SWING_POS_AE){
      return true;
    } else {
      return false;
    }
  }

  public Command manualSwingControl(){
    return run(() -> {
      selectedPosition += (m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis())*Constants.LAUNCHER_SWING_MAN_CTRL_SENS;

    });
  }

  public void init(){
    selectedPosition = swingEncoder.getDistance();
  }
  @Override
  public void periodic() {

    selectedPosition = clamp(selectedPosition, Constants.LAUNCHER_SWING_LOWER_BOUND, Constants.LAUNCHER_SWING_UPPER_BOUND);


    SmartDashboard.putNumber ("launcher swing selected pos", selectedPosition                );
    SmartDashboard.putNumber ("launcher swing current pos" , swingEncoder.getDistance()      );
    SmartDashboard.putBoolean("launcher swing encoder connected", swingEncoder.isConnected());
    

    switch(rollerState){
      case RECV:
        roller.set(ControlMode.PercentOutput, Constants.LAUNCHER_ROLLER_RECV_SPEED);
        break;
      case BACKOUT:
        roller.set(ControlMode.PercentOutput, Constants.LAUNCHER_ROLLER_BACKOUT_SPEED);
        break;
      case IDLE:
        roller.set(ControlMode.PercentOutput, 0);
        break;
    }

    //swingMotor.set(clamp(swingPID.calculate(swingEncoder.getDistance(), selectedPosition), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED));
  
    //bad pid
    swingMotor.set(clamp(3 * (selectedPosition - swingEncoder.getDistance()), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED));

  
  }
  
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}