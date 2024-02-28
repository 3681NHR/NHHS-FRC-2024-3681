// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

   private VictorSPX m_intakeBottom = new VictorSPX(Constants.INTAKE_BOTTOM_MOTOR_ID);
   private VictorSPX m_intakeTop    = new VictorSPX(Constants.INTAKE_TOP_MOTOR_ID   );
   private VictorSPX m_rotate       = new VictorSPX(Constants.INTAKE_SWING_MOTOR_ID );

   private double pidOut = 0.0;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.IDLE;

  private DutyCycleEncoder intakeSwingEncoder = new DutyCycleEncoder(Constants.INTAKE_SWING_ENCODER_DIO_PIN);
  //private PIDController swingPID = new PIDController(0, 0, 0);
  //pids are for nerds
  private double selectedPosition;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   //set motor idle modes
   this.m_intakeBottom.setNeutralMode(NeutralMode.Brake);
   this.m_intakeTop   .setNeutralMode(NeutralMode.Brake);
   this.m_rotate      .setNeutralMode(NeutralMode.Brake);
  }

  
  public double getPosition(boolean selectedPos){
    if(selectedPos){
      return selectedPosition;
    } else{
    return intakeSwingEncoder.getDistance();
    }
  }
  public void setPosition(double pos){
    selectedPosition = pos;
  }

  public boolean isAtSelectedPos(){
    if(Math.abs(selectedPosition - intakeSwingEncoder.getDistance()) <= Constants.INTAKE_SWING_POS_AE){
      return true;
    } else {
      return false;
    }
  }

  public void setIntake(IntakeState state){
    this.state = state;
  }
  public Command setIntakeCommand(IntakeState s){
    return runOnce(() -> {
      state = s;
    }
    );
  }
  

  public Command toggleSwing() {
    return runOnce(
      () -> {
      if(swingState == IntakeSwingState.UP){
        swingState = IntakeSwingState.DOWN;
      } else{
        swingState = IntakeSwingState.UP;
      }
      });
    }
    
    public Command toggleIntake() {
    return runOnce(
    () -> {
    if(state == IntakeState.INTAKE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.INTAKE; 
      } 
    });
  
  }

    public Command toggleReverse() {
    return runOnce(
    () -> {
    if(state == IntakeState.REVERSE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.REVERSE;
      } 
    });
    
  
  }  
    

  @Override
  public void periodic() {

    SmartDashboard.putNumber("intake swing selected pos" , selectedPosition                );
    SmartDashboard.putNumber("intake swing current pos"  , intakeSwingEncoder.getDistance());
    SmartDashboard.putString("intake swing state"        , swingState.toString()           );
    SmartDashboard.putString("intake state"              , state.toString()                );
    SmartDashboard.putNumber("intake swing \"PID\" value", pidOut                          );

    if(swingState == IntakeSwingState.UP){
      selectedPosition = Constants.INTAKE_SWING_UP_POSITION;
    }
    if(swingState == IntakeSwingState.DOWN){
      selectedPosition = Constants.INTAKE_SWING_DOWN_POSITION;
    }
    if(swingState == IntakeSwingState.IDLE){
      selectedPosition = intakeSwingEncoder.getDistance();
    }

    if(intakeSwingEncoder.getDistance() < selectedPosition){
    pidOut = clamp(6 * (selectedPosition - intakeSwingEncoder.getDistance()), -Constants.INTAKE_SWING_UP_SPEED,Constants.INTAKE_SWING_UP_SPEED);
    } else {
    pidOut = clamp(1 * (selectedPosition - intakeSwingEncoder.getDistance()), -Constants.INTAKE_SWING_DOWN_SPEED,Constants.INTAKE_SWING_DOWN_SPEED);
 
    }
    //pidOut = clamp(swingPID.calculate(intakeSwingEncoder.getDistance(),selectedPosition), -Constants.INTAKE_SWING_SPEED,Constants.INTAKE_SWING_SPEED);


    m_rotate.set(ControlMode.PercentOutput, pidOut);


  if(state == IntakeState.INTAKE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }else if(state == IntakeState.REVERSE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE_REVERSE_SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE_REVERSE_SPEED);    
  } else{
    m_intakeBottom.set(ControlMode.PercentOutput, 0);
    m_intakeTop   .set(ControlMode.PercentOutput, 0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));
  }
}

