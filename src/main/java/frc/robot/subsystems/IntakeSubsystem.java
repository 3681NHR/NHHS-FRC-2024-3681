// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.MotionType;
import frc.robot.interfaces.MotorInterface;
import frc.robot.wrappers.SparkWrapper;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class IntakeSubsystem extends SubsystemBase {

  private SparkWrapper m_intake;
  private SparkWrapper m_rotate;
  
  private IntakeState state = IntakeState.IDLE;



  
  private final double INTAKE_SPEED = 0.1;
  private final double ROTATE_SPEED = 0.1;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   this.m_intake = new SparkWrapper(1, MotorType.kBrushless);
   this.m_rotate = new SparkWrapper(1, MotorType.kBrushless);
   
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command toggleSwingUp() {
    return run(
      () -> {
      if(state == IntakeState.INTAKE){
        state = IntakeState.IDLE;
      } else{
        state = IntakeState.INTAKE;
      }
      });
    }
    public Command toggleSwingDown() {
    return run(
      () -> {
      if(state == IntakeState.INTAKE){
        state = IntakeState.IDLE;
      } else{
        state = IntakeState.INTAKE;
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
        state = IntakeState.INTAKE;
      } 
    });
  
  }  
  
  
  
  public Command rotate() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_rotate.setVelocity(ROTATE_SPEED);
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
