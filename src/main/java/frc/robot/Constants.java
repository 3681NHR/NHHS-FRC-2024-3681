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
    
  public static final int ASO_CONTROLLER_PORT                 = 0;
  public static final int DRIVER_CONTROLLER_PORT              = 1;

  public static final int DRIVE_FRONT_LEFT_MOTOR_ID           = 13;
  public static final int DRIVE_FRONT_RIGHT_MOTOR_ID          = 14;
  public static final int DRIVE_BACK_LEFT_MOTOR_ID            = 16;
  public static final int DRIVE_BACK_RIGHT_MOTOR_ID           = 15;

  public static final double DRIVE_INPUT_LIMITER              = 1;//watch out for sam h
  public static final double DRIVE_INPUT_DEADZONE             = 0.125;

  public static final double DRIVE_FAST_SPEED_MAX_INPUT       = 1;
  public static final double DRIVE_MEDIUM_SPEED_MAX_INPUT     = 0.5;
  public static final double DRIVE_SLOW_SPEED_MAX_INPUT       = 0.25;

  public static final int LAUNCHER_LEFT_MOTOR_ID              = 18;
  public static final int LAUNCHER_RIGHT_MOTOR_ID             = 17;

  public static final double LAUNCHER_LAUNCH_SPEED            = 1;
  public static final double LAUNCHER_DROP_SPEED              = 0.1;
  public static final double LAUNCHER_IN_SPEED                = -0.1;


  public static final double LAUNCHER_SWING_SPEED             = 1;
  public static final double LAUNCHER_ROLLER_RECV_SPEED       = -1;
  public static final double LAUNCHER_ROLLER_BACKOUT_SPEED    = 1;

  public static final double LAUNCHER_SWING_POS_AE            = 0.005;
  public static final double INTAKE_SWING_POS_AE              = 0.005;

  public static final double LAUNCHER_RECV_POSITION           = 0.50;
  public static final double LAUNCHER_LAUNCH_POSITION         = 0.56;
  public static final double LAUNCHER_DROP_POSITION           = 0.25;
 
  public static final double LAUNCHER_SWING_UPPER_BOUND       = 0.6;
  public static final double LAUNCHER_SWING_LOWER_BOUND       = 0.275;

  public static final int LAUNCHER_ROLLER_MOTOR_ID            = 6;
  public static final int LAUNCHER_SWING_MOTOR_ID             = 19;

  public static final double LAUNCHER_SWING_MAN_CTRL_SENS     = 0.01;

  public static final int LAUNCHER_SWING_ENCODER_DIO_PIN      = 9;

  public static final int INTAKE_TOP_MOTOR_ID                 = 7;
  public static final int INTAKE_BOTTOM_MOTOR_ID              = 8;

  public static final int INTAKE_SWING_MOTOR_ID               =5;

  public static final double INTAKE_SPEED                     =-1;
  public static final double INTAKE_REVERSE_SPEED             =1;

  public static final double INTAKE_SWING_UP_SPEED            =1;
  public static final double INTAKE_SWING_DOWN_SPEED          =0.5;

  public static final int INTAKE_SWING_ENCODER_DIO_PIN        =8;

  public static final int INTAKE_SWING_UPPER_BOUND            =1;
  public static final int INTAKE_SWING_LOWER_BOUND            =0;

  public static final double INTAKE_SWING_DOWN_POSITION       =0.3;
  public static final double INTAKE_SWING_UP_POSITION         =0.675;

}
