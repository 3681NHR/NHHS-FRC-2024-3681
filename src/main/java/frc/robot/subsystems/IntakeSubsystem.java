// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;


import frc.robot.wrappers.VictorWrapper;

import java.security.PublicKey;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IdleState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private VictorWrapper m_intakeBottom;
  private VictorWrapper m_intakeTop;
  private VictorWrapper m_rotate;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.DOWN;

  private Encoder intakeSwingEncoder = new Encoder(Constants.INTAKE_SWING_ENCODER_DIO_PIN_A, Constants.INTAKE_SWING_ENCODER_DIO_PIN_B);
  private DigitalInput homingSwitch = new DigitalInput(Constants.INTAKE_SWING_LIMIT_SWITCH_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);
  
  private double selectedPosition;

 

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   this.m_intakeBottom = new VictorWrapper(Constants.INTAKE_BOTTOM_MOTOR_ID);
   this.m_intakeTop = new VictorWrapper(Constants.INTAKE_TOP_MOTOR_ID);
   this.m_rotate = new VictorWrapper(Constants.INTAKE_SWING_MOTOR_ID);
   
   //set motor idle modes - did this *sam* 
   this.m_intakeBottom.setIdleMode(IdleState.BRAKE);
   this.m_intakeTop.setIdleMode(IdleState.BRAKE);
   this.m_rotate.setIdleMode(IdleState.BRAKE);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command toggleSwing() {
    return run(
      () -> {
      if(swingState == IntakeSwingState.UP){
        swingState = IntakeSwingState.DOWN;
      } else{
        swingState = IntakeSwingState.UP;
      }
      });
    }
    
    public Command toggleIntake() {
    return run(
    () -> {
    if(state == IntakeState.INTAKE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.INTAKE;
      } 
    });
  
  }

    public Command toggleReverse() {
    return run(
    () -> {
    if(state == IntakeState.REVERSE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.REVERSE;
      } 
    });
    
  
  }  
    public Command home() {
      return run(
      () -> {
      if(homingSwitch.get()){
        intakeSwingEncoder.reset();
        m_rotate.setVelocity(0);
      } else{
        m_rotate.setVelocity(Constants.INTAKE_SWING_HOMING_SPEED);
      }
    });
  
    }
    



  @Override
  public void periodic() {
    m_rotate.setVelocity(clamp(swingPID.calculate(intakeSwingEncoder.getDistance(),selectedPosition), -Constants.INTAKE_SWING_SPEED,Constants.INTAKE_SWING_SPEED));
  
  if(state == IntakeState.INTAKE){
    m_intakeBottom.setVelocity(Constants.INTAKE_SPEED);
    m_intakeTop.setVelocity(Constants.INTAKE_SPEED);
  }else if(state == IntakeState.REVERSE){
    m_intakeBottom.setVelocity(-Constants.INTAKE_SPEED);
    m_intakeTop.setVelocity(-Constants.INTAKE_SPEED);    
  } else{
    m_intakeBottom.setVelocity(0);
    m_intakeTop.setVelocity(0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));


  }
}

