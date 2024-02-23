// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left;
  private CANSparkMax m_back_right;
  private CANSparkMax m_front_left;
  private CANSparkMax m_front_right;

  double forward;
  double right; 
  double rotate;

  private Drive drive;

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);//change to DRIVER_CONTROLLER_PORT to use duel controller

  /** Creates a new Subsystem. */
  public DriveSubsystem() {
   this.m_back_left   = new CANSparkMax(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
   this.m_back_right  = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
   this.m_front_left  = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
   this.m_front_right = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
  
    //should be kOk if no error
   System.out.println(this.m_back_left  .setIdleMode(IdleMode.kBrake).toString());
   System.out.println(this.m_back_right .setIdleMode(IdleMode.kBrake).toString());
   System.out.println(this.m_front_left .setIdleMode(IdleMode.kBrake).toString());
   System.out.println(this.m_front_right.setIdleMode(IdleMode.kBrake).toString());

   this.m_back_right .setInverted(true);
   this.m_front_right.setInverted(true);

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

  }
  
  public Command Drive() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          drive.driveCartesian(forward, right, -rotate);
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    forward = limit(Constants.DRIVE_INPUT_LIMITER, deadzone(-m_driverController.getLeftY(),  Constants.DRIVE_INPUT_DEADZONE));
    right   = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getLeftX() , Constants.DRIVE_INPUT_DEADZONE));
    rotate  = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getRightX(), Constants.DRIVE_INPUT_DEADZONE));
    
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("right"  ,   right);
    SmartDashboard.putNumber("rotate" ,  rotate);
  }

  
  private double deadzone(double value, double zone)
  {
    double x = value;

    if(Math.abs(x)<zone){
      x=0;
    }
    return x;
  }
  private double limit(double lim, double  value)
  {
    if(lim  < 0)
    {
      return value;
    }
    else
    {
      if(Math.abs(value) <= lim){
        return value;
      }
      else
      {
        return lim * (Math.abs(value)/value);//multiply lim by normalized value(-1 or 1)
      }
    }
  }
}
