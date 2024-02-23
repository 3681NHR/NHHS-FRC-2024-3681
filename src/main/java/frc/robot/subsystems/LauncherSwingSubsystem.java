// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class LauncherSwingSubsystem extends SubsystemBase {

  private CANSparkMax swingMotor;

  private DutyCycleEncoder swingEncoder = new DutyCycleEncoder(Constants.LAUNCHER_SWING_ENCODER_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);

  private double selectedPosition;

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);

  public LauncherSwingSubsystem() 
  {

    this.swingMotor  = new CANSparkMax(Constants.LAUNCHER_SWING_MOTOR_ID, MotorType.kBrushless);

    this.swingMotor.setIdleMode(IdleMode.kBrake);

  }

  //public Command home(){
  //  return run(() -> {
  //    if(homingSwitch.get()){
  //      swingEncoder.reset();
  //      swingMotor.set(0);
  //    } else {
  //      swingMotor.set(Constants.LAUNCHER_SWING_HOMING_SPEED);
  //    }
  //  });
  //}

  //public void gotoPosition(double position){
  //  selectedPosition = position;
  //}
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

      selectedPosition = clamp(selectedPosition, Constants.LAUNCHER_SWING_LOWER_BOUND, Constants.LAUNCHER_SWING_UPPER_BOUND);
    });
  }
  @Override
  public void periodic() {

    SmartDashboard.putNumber ("launcher swing selected pos", selectedPosition                );
    SmartDashboard.putNumber ("launcher swing current pos" , swingEncoder.getDistance()      );
    SmartDashboard.putBoolean("launcher swing encoder connected", swingEncoder.isConnected());


    swingMotor.set(clamp(swingPID.calculate(swingEncoder.getDistance(), selectedPosition), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED));
  }
  
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}