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

  private final double[] SPEEDS_LAUNCH = {1, 1, 1, 1};
  private final double[] SPEEDS_DROP = {0.1, 0.1, 0.1, 0.1};


  /** Creates a new Subsystem. */
  public LauncherSubsystem() {
   this.m_back_left  = new SparkWrapper(1, MotorType.kBrushless);
   this.m_back_right = new SparkWrapper(1, MotorType.kBrushless);
   this.m_front_left = new SparkWrapper(1, MotorType.kBrushless);
   this.m_front_right= new SparkWrapper(1, MotorType.kBrushless);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command Startlaunch() {
    return run(
        () -> {
          
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
