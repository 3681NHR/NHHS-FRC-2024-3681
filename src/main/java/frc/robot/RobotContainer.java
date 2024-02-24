// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoRecv;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final LauncherSwingSubsystem m_LauncherSwingSubsystem = new LauncherSwingSubsystem();

  
  private final CommandXboxController m_commandDriverController = new CommandXboxController(Constants.ASO_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    m_driveSubsystem.setDefaultCommand(
      m_driveSubsystem.Drive()
    );
    m_LauncherSwingSubsystem.setDefaultCommand(
      m_LauncherSwingSubsystem.manualSwingControl()
    );


  }
  
  private void configureBindings() {

    m_commandDriverController.a().onTrue(m_launcherSubsystem.toggleLaunch());
    m_commandDriverController.b().onTrue(m_launcherSubsystem.toggleDrop());

    m_commandDriverController.rightBumper().onTrue(m_intakeSubsystem.toggleSwing());
    m_commandDriverController.leftBumper().onTrue(m_LauncherSwingSubsystem.toggleRollerRecv());
    
    m_commandDriverController.x().onTrue(m_intakeSubsystem.toggleIntake());
    m_commandDriverController.y().onTrue(m_intakeSubsystem.toggleReverse());

    m_commandDriverController.povUp()   .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_DROP_POSITION  ));
    m_commandDriverController.povLeft() .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_RECV_POSITION  ));
    m_commandDriverController.povRight().onTrue(new AutoRecv(m_intakeSubsystem, m_LauncherSwingSubsystem));
    m_commandDriverController.povDown() .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_LAUNCH_POSITION));
  
    m_commandDriverController.start().onTrue(m_LauncherSwingSubsystem.toggleRollerBackout());
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
