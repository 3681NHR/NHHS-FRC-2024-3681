// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class LauncherSubsystem extends SubsystemBase {

  private SparkWrapper m_back_left;
  private SparkWrapper m_back_right;
  private SparkWrapper m_front_left;
  private SparkWrapper m_front_right;

  private final double SPEED_LIM = 0.5;

  private double DEADZONE = 0.15;

  private Drive drive;

  /** Creates a new Subsystem. */
  public Subsystem() {
   this.m_back_left   = new SparkWrapper(1, MotorType.kBrushless);
   m_back_right  = new SparkWrapper(4, MotorType.kBrushless);
   m_front_left  = new SparkWrapper(3, MotorType.kBrushless);
   m_front_right = new SparkWrapper(2, MotorType.kBrushless);
   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command Drive(XboxController driverController) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          
          double forward = limit(SPEED_LIM, deadzone(-driverController.getLeftY() , DEADZONE));
          double right   = limit(SPEED_LIM, deadzone(driverController.getLeftX() , DEADZONE));
          double rotate  = limit(SPEED_LIM, deadzone(driverController.getRightX(), DEADZONE));
          
          SmartDashboard.putNumber("forward", forward);
          SmartDashboard.putNumber("right"  ,   right);
          SmartDashboard.putNumber("rotate" ,  rotate);

          drive.driveCartesian(rotate, right, forward);

          //this.setDrive(
          //  forward - right - rotate, 
          //  forward + right + rotate, 
          //  forward - right + rotate, 
          //  forward + right - rotate
          //);
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

  public void clearDrive(){
    m_back_left.set  (MotionType.VELOCITY, 0.0);
    m_back_right.set (MotionType.VELOCITY, 0.0);
    m_front_left.set (MotionType.VELOCITY, 0.0);
    m_front_right.set(MotionType.VELOCITY, 0.0);
  }

  public void setDrive(double front_left, double front_right, double back_left, double back_right){
    m_front_left .set (MotionType.VELOCITY, front_left );
    m_front_right.set (MotionType.VELOCITY, front_right);
    m_back_left  .set (MotionType.VELOCITY, back_left  );
    m_back_right .set (MotionType.VELOCITY, back_right );
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
