// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.LauncherState;
import frc.robot.enums.MotionType;
import frc.robot.interfaces.MotorInterface;
import frc.robot.wrappers.SparkWrapper;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class LauncherSubsystem extends SubsystemBase {

  private SparkWrapper m_back_left;
  private SparkWrapper m_back_right;
  private SparkWrapper m_front_left;
  private SparkWrapper m_front_right;

  private final double[] SPEEDS_LAUNCH = {1, 1, 1, 1};
  private final double[] SPEEDS_DROP = {0.1, 0.1, 0.1, 0.1};


  public LauncherState state = LauncherState.IDLE;

  public LauncherSubsystem() {
   this.m_back_left  = new SparkWrapper(1, MotorType.kBrushless);
   this.m_back_right = new SparkWrapper(1, MotorType.kBrushless);
   this.m_front_left = new SparkWrapper(1, MotorType.kBrushless);
   this.m_front_right= new SparkWrapper(1, MotorType.kBrushless);


    this.m_back_left  .setIdleMode(IdleMode.kCoast);
    this.m_back_right .setIdleMode(IdleMode.kCoast);
    this.m_front_left .setIdleMode(IdleMode.kCoast);
    this.m_front_right.setIdleMode(IdleMode.kCoast);
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

  public void Startlaunch() {
    m_back_left  .setVelocity(SPEEDS_LAUNCH[0]);
    m_back_right .setVelocity(SPEEDS_LAUNCH[1]);
    m_front_left .setVelocity(SPEEDS_LAUNCH[2]);
    m_front_right.setVelocity(SPEEDS_LAUNCH[3]);
  }
  public void StartDrop() {
    m_back_left  .setVelocity(SPEEDS_DROP[0]);
    m_back_right .setVelocity(SPEEDS_DROP[1]);
    m_front_left .setVelocity(SPEEDS_DROP[2]);
    m_front_right.setVelocity(SPEEDS_DROP[3]);
  }
  public void EndMotion() {
    
    m_back_left  .setVelocity(0);
    m_back_right .setVelocity(0);
    m_front_left .setVelocity(0);
    m_front_right.setVelocity(0);

  }

  @Override
  public void periodic() {
    
    switch(state){
      case LAUNCHING:
        m_back_left  .setVelocity(SPEEDS_LAUNCH[0]);
        m_back_right .setVelocity(SPEEDS_LAUNCH[1]);
        m_front_left .setVelocity(SPEEDS_LAUNCH[2]);
        m_front_right.setVelocity(SPEEDS_LAUNCH[3]);
        break;
      case DROPPING:
        m_back_left  .setVelocity(SPEEDS_DROP[0]);
        m_back_right .setVelocity(SPEEDS_DROP[1]);
        m_front_left .setVelocity(SPEEDS_DROP[2]);
        m_front_right.setVelocity(SPEEDS_DROP[3]);
        break;
      case IDLE:
        m_back_left  .setVelocity(0);
        m_back_right .setVelocity(0);
        m_front_left .setVelocity(0);
        m_front_right.setVelocity(0);
        break;
    }
  }

}
