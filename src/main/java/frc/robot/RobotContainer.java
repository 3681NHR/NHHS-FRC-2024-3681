// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto;
import frc.robot.commands.AutoLaunchOnly;
import frc.robot.enums.IntakeState;
import frc.robot.enums.LauncherState;
import frc.robot.enums.RobotPosition;
import frc.robot.enums.RollerState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherSwingSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final LauncherSwingSubsystem m_LauncherSwingSubsystem = new LauncherSwingSubsystem();

  
  private final CommandXboxController m_commandASOController = new CommandXboxController(Constants.ASO_CONTROLLER_PORT);
  private final CommandXboxController m_commandDriverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);


  private Command m_fullAuto       = new Auto(m_LauncherSwingSubsystem, m_launcherSubsystem, m_driveSubsystem);
  private Command m_AutoLaunchOnly = new AutoLaunchOnly(m_LauncherSwingSubsystem, m_launcherSubsystem);

  private SendableChooser<Command> Autos = new SendableChooser<>();
  private SendableChooser<RobotPosition>  startingPositions = new SendableChooser<>();

  private RobotPosition center = new RobotPosition(0.0, 0.0, 180.0);
  private RobotPosition left   = new RobotPosition(0.0, 0.0, 180.0);
  private RobotPosition right  = new RobotPosition(0.0, 0.0, 180.0);
  private RobotPosition zero  = new RobotPosition(0.0, 0.0, 0.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    m_LauncherSwingSubsystem.setDefaultCommand(
      m_LauncherSwingSubsystem.manualSwingControl()
    );
    
    configureBindings();

    Autos.setDefaultOption("full", m_fullAuto);
    Autos.addOption("no reverse" , m_AutoLaunchOnly);
    Autos.addOption("no auto", null);

    startingPositions.setDefaultOption("center", center);
    startingPositions.addOption("left", left);
    startingPositions.addOption("right", right);
    startingPositions.addOption("forward", zero);

    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_intakeSubsystem);
    SmartDashboard.putData(m_launcherSubsystem);
    SmartDashboard.putData(m_LauncherSwingSubsystem);

    SmartDashboard.putData(Autos);  
    SmartDashboard.putData(startingPositions);
  }
  
  private void configureBindings() {//keybindings

    m_commandASOController.a().onTrue(m_launcherSubsystem.setSpeedCommand(LauncherState.LAUNCHING));
    m_commandASOController.b().onTrue(m_launcherSubsystem.setSpeedCommand(LauncherState.DROPPING));

    m_commandASOController.a().onFalse(m_launcherSubsystem.setSpeedCommand(LauncherState.IDLE));
    m_commandASOController.b().onFalse(m_launcherSubsystem.setSpeedCommand(LauncherState.IDLE));

    m_commandASOController.rightBumper().onTrue(m_intakeSubsystem.toggleSwing());
    m_commandASOController.leftBumper().onTrue(m_LauncherSwingSubsystem.setRollerCommand(RollerState.RECV));
    m_commandASOController.leftBumper().onFalse(m_LauncherSwingSubsystem.setRollerCommand(RollerState.IDLE));
    
    m_commandASOController.x().onTrue(m_intakeSubsystem.setIntakeCommand(IntakeState.INTAKE));
    m_commandASOController.y().onTrue(m_intakeSubsystem.setIntakeCommand(IntakeState.REVERSE));

    m_commandASOController.x().onFalse(m_intakeSubsystem.setIntakeCommand(IntakeState.IDLE));
    m_commandASOController.y().onFalse(m_intakeSubsystem.setIntakeCommand(IntakeState.IDLE));

    m_commandASOController.povUp()   .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_DROP_POSITION  ));
    m_commandASOController.povLeft() .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_RECV_POSITION  ));
    //m_commandDriverController.povRight().onTrue(new AutoRecv(m_intakeSubsystem, m_LauncherSwingSubsystem));
    m_commandASOController.povDown() .onTrue(m_LauncherSwingSubsystem.setPositionCommand(Constants.LAUNCHER_LAUNCH_POSITION));
  
    m_commandASOController.start().onTrue(m_LauncherSwingSubsystem.setRollerCommand(RollerState.BACKOUT));
    m_commandASOController.start().onFalse(m_LauncherSwingSubsystem.setRollerCommand(RollerState.IDLE));

    m_commandASOController.back().onTrue (m_launcherSubsystem.setSpeedCommand(LauncherState.IN));
    m_commandASOController.back().onFalse(m_launcherSubsystem.setSpeedCommand(LauncherState.IDLE));

    m_commandDriverController.a().onTrue(m_driveSubsystem.toggleFOD());
    m_commandDriverController.b().onTrue(m_driveSubsystem.zero());
    m_commandDriverController.x().onTrue(m_driveSubsystem.toggleModeChanging());
    m_commandDriverController.y().onTrue(m_driveSubsystem.togglesquaring());

    m_commandDriverController.povUp()   .whileTrue(m_driveSubsystem.sysidDynamic(Direction.kForward));
    m_commandDriverController.povDown() .whileTrue(m_driveSubsystem.sysidDynamic(Direction.kReverse));
    m_commandDriverController.povLeft() .whileTrue(m_driveSubsystem.sysidQuasistatic(Direction.kForward));
    m_commandDriverController.povRight().whileTrue(m_driveSubsystem.sysidQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.getSelected();
  }
  public void disabledPeriodic(){
    m_LauncherSwingSubsystem.disabledPeriodic();
  }
  public void teleopPeriodic(){
    m_driveSubsystem.teleopPeriodic();

  }
}
