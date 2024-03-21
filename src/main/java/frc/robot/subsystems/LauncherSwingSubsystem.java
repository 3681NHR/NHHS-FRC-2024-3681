// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSwingSubsystem extends SubsystemBase {

  private CANSparkMax swingMotor  = new CANSparkMax(Constants.LAUNCHER_SWING_MOTOR_ID, MotorType.kBrushless);

  private double PIDOut;

  private DutyCycleEncoder swingEncoder = new DutyCycleEncoder(Constants.LAUNCHER_SWING_ENCODER_DIO_PIN);
  private ProfiledPIDController swingPID = new ProfiledPIDController(
    Constants.LAUNCHER_SWING_P_GAIN,
    Constants.LAUNCHER_SWING_I_GAIN,
    Constants.LAUNCHER_SWING_D_GAIN,
    new Constraints(10, 20)
  );

  //pid is eh
  private double selectedPosition;

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);


  public LauncherSwingSubsystem() 
  {

    swingPID.setTolerance(Constants.LAUNCHER_SWING_POS_AE, Constants.LAUNCHER_SWING_PID_VELOCITY_TOLERANCE);

    swingPID.setIntegratorRange(-1, 1);

    SmartDashboard.putNumber ("pid P gain", swingPID.getP());
    SmartDashboard.putNumber ("pid I gain", swingPID.getI());
    SmartDashboard.putNumber ("pid D gain", swingPID.getD());
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
    return run(() -> {
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

  public Command manualSwingControlCommand(){
    return run(() -> {
      selectedPosition += (m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis())*Constants.LAUNCHER_SWING_MAN_CTRL_SENS;

    });
  }

  public void disabledPeriodic(){
    selectedPosition = swingEncoder.getDistance();
  }
  @Override
  public void periodic() {

    this.swingMotor.setIdleMode(IdleMode.kBrake);


    selectedPosition = clamp(selectedPosition, Constants.LAUNCHER_SWING_LOWER_BOUND, Constants.LAUNCHER_SWING_UPPER_BOUND);

    PIDOut = clamp(swingPID.calculate(swingEncoder.getDistance(), selectedPosition), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED);

    SmartDashboard.putNumber ("launcher swing selected pos"     , selectedPosition          );
    SmartDashboard.putNumber ("launcher swing current pos"      , swingEncoder.getDistance());
    SmartDashboard.putBoolean("launcher swing encoder connected", swingEncoder.isConnected());
    SmartDashboard.putNumber ("pid out", PIDOut);

    swingPID.setP(SmartDashboard.getNumber("pid P gain", Constants.LAUNCHER_SWING_P_GAIN));
    swingPID.setI(SmartDashboard.getNumber("pid I gain", Constants.LAUNCHER_SWING_I_GAIN));
    swingPID.setD(SmartDashboard.getNumber("pid D gain", Constants.LAUNCHER_SWING_D_GAIN));

    
    //PID controller
    swingMotor.set(PIDOut);
    //P controller
    //swingMotor.set(clamp(3 * (selectedPosition - swingEncoder.getDistance()), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED));

  
  }
  
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}