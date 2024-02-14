// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
  public static final int DRIVER_CONTROLLER_PORT              = 0;

  public static final int DRIVE_FRONT_LEFT_MOTOR_ID           = 15;
  public static final int DRIVE_FRONT_RIGHT_MOTOR_ID          = 16;
  public static final int DRIVE_BACK_LEFT_MOTOR_ID            = 14;
  public static final int DRIVE_BACK_RIGHT_MOTOR_ID           = 13;

  public static final double DRIVE_INPUT_LIMITER              = 0.5;
  public static final double DRIVE_INPUT_DEADZONE             = 0.15;

  public static final int LAUNCHER_LEFT_MOTOR_ID              = 17;
  public static final int LAUNCHER_RIGHT_MOTOR_ID             = 18;

  public static final double LAUNCHER_LAUNCH_SPEED            = 1;
  public static final double LAUNCHER_DROP_SPEED              = 0.1;

  public static final double LAUNCHER_SWING_SPEED             = 0.1;
  public static final double LAUNCHER_SWING_HOMING_SPEED      = 0.1;

  public static final int LAUNCHER_STOW_POSITION              = 0;
  public static final int LAUNCHER_LAUNCH_POSITION            = 0;
  public static final int LAUNCHER_DROP_POSITION              = 0;
 
  public static final int LAUNCHER_SWING_UPPER_BOUND          = 1000;
  public static final int LAUNCHER_SWING_LOWER_BOUND          = -1000;

  public static final int LAUNCHER_SWING_MOTOR_ID             = 12;

  public static final int LAUNCHER_SWING_ENCODER_DIO_PIN_A    = 0;
  public static final int LAUNCHER_SWING_ENCODER_DIO_PIN_B    = 1;

  public static final int LAUNCHER_SWING_LIMIT_SWITCH_DIO_PIN = 2;

  public static final int INTAKE_TOP_MOTOR_ID                 = 5;
  public static final int INTAKE_BOTTOM_MOTOR_ID              = 6;

  public static final int INTAKE_SWING_MOTOR_ID               =7;

  public static final double INTAKE_SPEED                     =1;
  public static final double INTAKE_REVERSE_SPEED             =-1;
  public static final double INTAKE_SWING_SPEED               =0.5;

  public static final int INTAKE_SWING_ENCODER_DIO_PIN_A      =0;
  public static final int INTAKE_SWING_ENCODER_DIO_PIN_B      =1;

  public static final int INTAKE_SWING_LIMIT_SWITCH_DIO_PIN   =0;

  public static final int INTAKE_SWING_UPPER_BOUND            =1000;
  public static final int INTAKE_SWING_LOWER_BOUND            =-1000;

  public static final double INTAKE_SWING_HOMING_SPEED        =0.1;

  public static final int INTAKE_SWING_DOWN_POSITION          =0;
  public static final int INTAKE_SWING_UP_POSITION            =0;

}
