// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IdleState;
import frc.robot.enums.LauncherState;
import frc.robot.wrappers.SparkWrapper;
import frc.robot.Constants;

public class LauncherSwingSubsystem extends SubsystemBase {

  private SparkWrapper swingMotor;

  private Encoder swingEncoder = new Encoder(Constants.LAUNCHER_SWING_ENCODER_DIO_PIN_A, Constants.LAUNCHER_SWING_ENCODER_DIO_PIN_B);
  private DigitalInput homingSwitch = new DigitalInput(Constants.LAUNCHER_SWING_LIMIT_SWITCH_DIO_PIN);
  private PIDController swingPID = new PIDController(0, 0, 0);

  private double selectedPosition;

  private XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  public LauncherState state = LauncherState.IDLE;

  public LauncherSwingSubsystem() 
  {

    this.swingMotor  = new SparkWrapper(Constants.LAUNCHER_SWING_MOTOR_ID, MotorType.kBrushless);

    this.swingMotor.setIdleMode(IdleState.BRAKE);

  }

  public Command home(){
    return run(() -> {
      if(homingSwitch.get()){
        swingEncoder.reset();
        swingMotor.setVelocity(0);
      } else {
        swingMotor.setVelocity(Constants.LAUNCHER_SWING_HOMING_SPEED);
      }
    });
  }

  public Command manualSwingControl(){
    return runOnce(() -> {
      selectedPosition += m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis();

      selectedPosition = clamp(selectedPosition, Constants.LAUNCHER_SWING_LOWER_BOUND, Constants.LAUNCHER_SWING_UPPER_BOUND);
    });
  }
  @Override
  public void periodic() {
    swingMotor.setVelocity(clamp(swingPID.calculate(swingEncoder.getDistance(), selectedPosition), -Constants.LAUNCHER_SWING_SPEED, Constants.LAUNCHER_SWING_SPEED));
  }
  
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}