// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final LauncherSwingSubsystem m_LauncherSwingSubsystem = new LauncherSwingSubsystem();

  
  private final CommandXboxController m_commandDriverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    m_driveSubsystem.setDefaultCommand(
      m_driveSubsystem.Drive()
    );

    CommandScheduler.getInstance().schedule(m_LauncherSwingSubsystem.home());

  }
  
  private void configureBindings() {

    m_commandDriverController.a().onTrue(m_launcherSubsystem.toggleLaunch());
    m_commandDriverController.b().onTrue(m_launcherSubsystem.toggleDrop());

    m_commandDriverController.rightBumper().onTrue(m_intakeSubsystem.toggleSwing());
    

    m_commandDriverController.x().onTrue(m_intakeSubsystem.toggleIntake());
    m_commandDriverController.y().onTrue(m_intakeSubsystem.toggleReverse());

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
