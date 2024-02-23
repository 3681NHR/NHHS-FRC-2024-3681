// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private VictorSPX m_intakeBottom;
  private VictorSPX m_intakeTop;
  private VictorSPX m_rotate;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.DOWN;

  private Encoder intakeSwingEncoder = new Encoder(Constants.INTAKE_SWING_ENCODER_DIO_PIN_A, Constants.INTAKE_SWING_ENCODER_DIO_PIN_B);
  private DigitalInput homingSwitch = new DigitalInput(Constants.INTAKE_SWING_LIMIT_SWITCH_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);
  
  private double selectedPosition;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   this.m_intakeBottom = new VictorSPX(Constants.INTAKE_BOTTOM_MOTOR_ID);
   this.m_intakeTop    = new VictorSPX(Constants.INTAKE_TOP_MOTOR_ID   );
   this.m_rotate       = new VictorSPX(Constants.INTAKE_SWING_MOTOR_ID );
   
   //set motor idle modes - did this *sam* 
   this.m_intakeBottom.setNeutralMode(NeutralMode.Brake);
   this.m_intakeTop   .setNeutralMode(NeutralMode.Brake);
   this.m_rotate      .setNeutralMode(NeutralMode.Brake);
  }

  
  public Command gotostate(int swing, int intake){
    return runOnce(() -> {
      switch(swing){
        case 0:
          swingState = IntakeSwingState.UP;
          break;
        case 1:
          swingState = IntakeSwingState.DOWN;
          break;
        default:
          throw new IndexOutOfBoundsException("intake swing goto set to bad position index");
      }
      switch(intake){
        case 0:
          state = IntakeState.IDLE;
          break;
        case 1:
          state = IntakeState.INTAKE;
          break;
        case 2:
          state = IntakeState.REVERSE;
          break;
        default:
          throw new IndexOutOfBoundsException("intake goto set to bad motion index");
      }

    });
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
    public Command home() {
      return run(
      () -> {
      if(homingSwitch.get()){
        intakeSwingEncoder.reset();
        m_rotate.set(ControlMode.Velocity, 0);
      } else{
        m_rotate.set(ControlMode.Velocity, Constants.INTAKE_SWING_HOMING_SPEED);

        //add interupt to end homing
      }
    });
  
    }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("intake swing selected pos", selectedPosition                );
    SmartDashboard.putNumber("intake swing current pos" , intakeSwingEncoder.getDistance());
    SmartDashboard.putString("intake swing state"       , swingState.toString()           );
    SmartDashboard.putString("intake state"             , state.toString()                );


    if(swingState == IntakeSwingState.UP){
      selectedPosition = Constants.INTAKE_SWING_UP_POSITION;
    }
    if(swingState == IntakeSwingState.DOWN){
      selectedPosition = Constants.INTAKE_SWING_DOWN_POSITION;
    }

    m_rotate.set(ControlMode.Velocity, clamp(swingPID.calculate(intakeSwingEncoder.getDistance(),selectedPosition), -Constants.INTAKE_SWING_SPEED,Constants.INTAKE_SWING_SPEED));
  
  if(state == IntakeState.INTAKE){
    m_intakeBottom.set(ControlMode.Velocity, Constants.INTAKE_SPEED);
    m_intakeTop   .set(ControlMode.Velocity, Constants.INTAKE_SPEED);
  }else if(state == IntakeState.REVERSE){
    m_intakeBottom.set(ControlMode.Velocity, Constants.INTAKE_REVERSE_SPEED);
    m_intakeTop   .set(ControlMode.Velocity, Constants.INTAKE_REVERSE_SPEED);    
  } else{
    m_intakeBottom.set(ControlMode.Velocity, 0);
    m_intakeTop   .set(ControlMode.Velocity, 0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));
  }
}

