// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IdleState;
import frc.robot.enums.LauncherState;
import frc.robot.wrappers.SparkWrapper;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private SparkWrapper m_left;
  private SparkWrapper m_right;

  public LauncherState state = LauncherState.IDLE;

  public LauncherSubsystem() {
   this.m_left  = new SparkWrapper(Constants.LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
   this.m_right = new SparkWrapper(Constants.LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);


    this.m_left  .setIdleMode(IdleState.BRAKE);
    this.m_right .setIdleMode(IdleState.BRAKE);
  }

  public Command toggleLaunch(){
    return runOnce(() -> {
      if(state == LauncherState.LAUNCHING){
        state = LauncherState.IDLE;
      } else{
        state = LauncherState.LAUNCHING;
      }
    });
  }
  public Command toggleDrop(){
    return runOnce(() -> {
      if(state == LauncherState.DROPPING){
        state = LauncherState.IDLE;
      } else{
        state = LauncherState.DROPPING;
      }
    });
  }
  
  @Override
  public void periodic() {
    switch(state){
      case LAUNCHING:
        m_left  .setVelocity(Constants.LAUNCHER_LAUNCH_SPEED);
        m_right .setVelocity(Constants.LAUNCHER_LAUNCH_SPEED);
        break;
      case DROPPING:
        m_left  .setVelocity(Constants.LAUNCHER_DROP_SPEED);
        m_right .setVelocity(Constants.LAUNCHER_DROP_SPEED);
        break;
      case IDLE:
        m_left  .setVelocity(0);
        m_right .setVelocity(0);
        break;
    }
  }

}
