// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSwingSubsystem extends SubsystemBase {

  private CANSparkMax swingMotor  = new CANSparkMax(Constants.LAUNCHER_SWING.MOTOR_ID, MotorType.kBrushless);

  private double PIDOut;
  private double feedforwardOut;

  private DutyCycleEncoder swingEncoder = new DutyCycleEncoder(Constants.LAUNCHER_SWING.ENCODER_DIO_PIN);
  private ProfiledPIDController swingPID = new ProfiledPIDController(
    Constants.LAUNCHER_SWING.P_GAIN,
    Constants.LAUNCHER_SWING.I_GAIN,
    Constants.LAUNCHER_SWING.D_GAIN,
    new Constraints(10, 20)
  );
  private ArmFeedforward feedforward = new ArmFeedforward(
    Constants.LAUNCHER_SWING.FEEDFORWARD_S_GAIN, 
    Constants.LAUNCHER_SWING.FEEDFORWARD_G_GAIN, 
    Constants.LAUNCHER_SWING.FEEDFORWARD_V_GAIN
    );

  private MutableMeasure<Angle> selectedAngle;

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);


  public LauncherSwingSubsystem() 
  {

    swingPID.setIntegratorRange(-1, 1);

    swingEncoder.setDistancePerRotation(2*Math.PI);
    swingEncoder.setPositionOffset(0);

  }
  
  public Measure<Angle> getPosition(boolean selectedAngle){
    if(selectedAngle){
      return this.selectedAngle;
    } else{
    return Radians.of(swingEncoder.getDistance());
    }
  }

  public void setAngle(Measure<Angle> pos){
    selectedAngle = (MutableMeasure<Angle>) pos;
  }

  public Command setAngleCommand(Measure<Angle> pos){
    return runOnce(() -> {
      selectedAngle = (MutableMeasure<Angle>) pos;
    });
  }

  public boolean isAtSelectedPos(){
    if(Math.abs(selectedAngle.magnitude() - swingEncoder.getDistance()) <= Constants.LAUNCHER_SWING.POS_AE){
      return true;
    } else {
      return false;
    }
  }

  public Command manualSwingControlCommand(){
    return run(() -> {
      selectedAngle.mut_acc((m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis())*Constants.LAUNCHER_SWING.MAN_CTRL_SENS);
    });
  }

  public void disabledPeriodic(){
    selectedAngle = (MutableMeasure<Angle>) Radians.of(swingEncoder.getDistance());
  }
  @Override
  public void periodic() {

    this.swingMotor.setIdleMode(IdleMode.kBrake);
    
    selectedAngle.mut_setMagnitude(clamp(selectedAngle.magnitude(), Constants.LAUNCHER_SWING.LOWER_BOUND, Constants.LAUNCHER_SWING.UPPER_BOUND));


    PIDOut = clamp(swingPID.calculate(swingEncoder.getDistance(), selectedAngle.magnitude()), -Constants.LAUNCHER_SWING.SPEED, Constants.LAUNCHER_SWING.SPEED);
    feedforwardOut = feedforward.calculate(swingPID.getSetpoint().position, swingPID.getSetpoint().velocity);

    SmartDashboard.putNumber ("launcher swing selected pos"     , selectedAngle.magnitude() );
    SmartDashboard.putNumber ("launcher swing current pos"      , swingEncoder.getDistance());
    SmartDashboard.putNumber ("pid out", PIDOut);
    
    //PID controller
    swingMotor.setVoltage(PIDOut + feedforwardOut);
  
  }
  
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}