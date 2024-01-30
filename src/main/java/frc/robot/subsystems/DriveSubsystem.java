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
import frc.robot.enums.MotionType;
import frc.robot.interfaces.MotorInterface;
import frc.robot.wrappers.SparkWrapper;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

  private SparkWrapper m_back_left;
  private SparkWrapper m_back_right;
  private SparkWrapper m_front_left;
  private SparkWrapper m_front_right;

  double forward;
  double right; 
  double rotate;

  private final double SPEED_LIM = 0.5;

  private double DEADZONE = 0.15;

  private Drive drive;

  /** Creates a new Subsystem. */
  public DriveSubsystem() {
   this.m_back_left   = new SparkWrapper(12, MotorType.kBrushless);
   this.m_back_right  = new SparkWrapper(13, MotorType.kBrushless);
   this.m_front_left  = new SparkWrapper(14, MotorType.kBrushless);
   this.m_front_right = new SparkWrapper(15, MotorType.kBrushless);
  
    
   this.m_back_left  .setIdleMode(IdleMode.kCoast);
   this.m_back_right .setIdleMode(IdleMode.kCoast);
   this.m_front_left .setIdleMode(IdleMode.kCoast);
   this.m_front_right.setIdleMode(IdleMode.kCoast);

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

  }
  
  public Command Drive(XboxController driverController) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {

          drive.driveCartesian(rotate, right, forward);

        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    forward = limit(SPEED_LIM, deadzone(-driverController.getLeftY(), DEADZONE));
    right   = limit(SPEED_LIM, deadzone(driverController.getLeftX() , DEADZONE));
    rotate  = limit(SPEED_LIM, deadzone(driverController.getRightX(), DEADZONE));
    
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
