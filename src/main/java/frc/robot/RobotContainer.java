// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class RobotContainer {
  private final Field2d m_field = new Field2d();

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();
  
  // private final CommandXboxController m_commandDriverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    m_driveSubsystem.setDefaultCommand(
      m_driveSubsystem.Drive()
    );

    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    m_driveSubsystem.setField(m_field);

  }
  
  private void configureBindings() {

    // m_commandDriverController.a().onTrue(m_launcherSubsystem.toggleLaunch());
    // m_commandDriverController.b().onTrue(m_launcherSubsystem.toggleDrop());

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }


  // Adding this in as a helper for testing simulation; 
  // This breaks our plans but we should refactor once we know how it works. 
  public CANSparkMax getFLMotor(){
    return this.m_driveSubsystem.getFLCanSparkMax();
  }
  public CANSparkMax getBLMotor(){
    return this.m_driveSubsystem.getBLCanSparkMax();
  }
  public CANSparkMax getFRMotor(){
    return this.m_driveSubsystem.getFRCanSparkMax();
  }
  public CANSparkMax getBRMotor(){
    return this.m_driveSubsystem.getBRCanSparkMax();
  }
}
