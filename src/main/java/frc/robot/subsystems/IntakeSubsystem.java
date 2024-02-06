// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;


import frc.robot.wrappers.VictorWrapper;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private VictorWrapper m_intakeBottom;
  private VictorWrapper m_intakeTop;
  private VictorWrapper m_rotate;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.DOWN;

  private Encoder intakeEncoder = new Encoder(Constants.INTAKE_SWING_ENCODER_DIO_PIN_A, Constants.INTAKE_SWING_ENCODER_DIO_PIN_B);
  private DigitalInput homingSwitch = new DigitalInput(Constants.INTAKE_SWING_LIMIT_SWITCH_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);
  
  private double selectedPosition;
  


  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   this.m_intakeBottom = new VictorWrapper(Constants.INTAKE_BOTTOM_MOTOR_ID);
   this.m_intakeTop = new VictorWrapper(Constants.INTAKE_TOP_MOTOR_ID);
   this.m_rotate = new VictorWrapper(Constants.INTAKE_SWING_MOTOR_ID);
   
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
  
  
  
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



 
  }
