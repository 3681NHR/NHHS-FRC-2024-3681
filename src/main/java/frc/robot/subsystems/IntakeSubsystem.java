// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.wrappers.VictorWrapper;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.wrappers.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private VictorWrapper m_intake;
  private VictorWrapper m_rotate;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.DOWN;


  
  private final double INTAKE_SPEED = 0.1;
  private final double ROTATE_SPEED = 0.1;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   this.m_intake = new VictorWrapper(1);
   this.m_rotate = new VictorWrapper(1);
   
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
