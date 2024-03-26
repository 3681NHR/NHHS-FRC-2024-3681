// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.*;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DRIVE{
    public static final int FRONT_LEFT_MOTOR_ID           = 13;
    public static final int FRONT_RIGHT_MOTOR_ID          = 14;
    public static final int BACK_LEFT_MOTOR_ID            = 16;
    public static final int BACK_RIGHT_MOTOR_ID           = 15;

    public static final double INPUT_LIMITER              = 1;//limit before remap, just use 1
    public static final double INPUT_DEADZONE             = 0.13;

    public static final double FAST_SPEED_MAX_INPUT       = 1;//these will be the max after remap
    public static final double MEDIUM_SPEED_MAX_INPUT     = 0.5;
    public static final double SLOW_SPEED_MAX_INPUT       = 0.25;

    public static final double ROTATE_P_GAIN              = 0.0;
    public static final double ROTATE_I_GAIN              = 0.0;
    public static final double ROTATE_D_GAIN              = 0.0;
  }
    
  public static final int ASO_CONTROLLER_PORT                 = 0;
  public static final int DRIVER_CONTROLLER_PORT              = 1;

  public static final class LAUNCHER{
    public static final int LEFT_MOTOR_ID              = 18;
    public static final int RIGHT_MOTOR_ID             = 17;

    public static final double LAUNCH_OUTPUT            = 1;
    public static final double DROP_OUTPUT              = 0.3;
    public static final double IN_OUTPUT                = -0.075;

    public static final Measure<Velocity<Angle>> LAUNCH_SPEED_RPM= RPM.of(4200);
    public static final Measure<Velocity<Angle>> DROP_SPEED_RPM  = RPM.of(1000);
  }
  
  public static final class INDEXER{
    public static final int MOTOR_ID            = 6;

    public static final double RECV_OUTPUT       = -1;
    public static final double BACKOUT_OUTPUT    = 0.75;
     
    public static final int DETECTOR_DIO_PIN           =7;
  }

  public static final class LAUNCHER_SWING{
    public static final Measure<Voltage> MAX_OUTPUT     = Volt.of(15);

    public static final Measure<Angle> POS_AE            = Rotation.of(0.015);

    public static final Measure<Angle> RECV_POSITION  = Rotation.of(0.502);
    public static final Measure<Angle> LAUNCH_POSITION= Rotation.of(0.55);
    public static final Measure<Angle> DROP_POSITION  = Rotation.of(0.275);

    public static final Measure<Angle> UPPER_BOUND       = Rotation.of(0.578);
    public static final Measure<Angle> LOWER_BOUND       = Rotation.of(0.275);

    public static final double P_GAIN            =25;
    public static final double I_GAIN            =0.0;
    public static final double D_GAIN            =0.0;

    public static final double FEEDFORWARD_S_GAIN = 0.0;
    public static final double FEEDFORWARD_G_GAIN = 0.0;
    public static final double FEEDFORWARD_V_GAIN = 0.0;

    public static final int    MOTOR_ID             = 19;
    public static final int    ENCODER_DIO_PIN      = 9;
    public static final double MAN_CTRL_SENS     = 0.01;
  }

  public static final class INTAKE{
    public static final int    TOP_MOTOR_ID                 = 7;
    public static final int    BOTTOM_MOTOR_ID              = 8;

    public static final double INTAKE_OUTPUT                     =1;
    public static final double REVERSE_OUTPUT             =-1;

    public static final int    DETECTOR_DIO_PIN             =6;
  }

  public static final class INTAKE_SWING{
    public static final int    MOTOR_ID               =9;

    public static final double MAX_OUTPUT               =0.5;

    public static final int    ENCODER_DIO_PIN        =8;
    
    public static final Measure<Angle> DOWN_POSITION       = Rotation.of(0.685);
    public static final Measure<Angle> UP_POSITION         = Rotation.of(1.116);

    public static final Measure<Angle> PID_SWITCH          =Rotation.of(0.2);

    public static final double P_GAIN              =4;//2
    public static final double I_GAIN              =0.0;//0.1
    public static final double D_GAIN              =0.0;

    public static final double POS_AE              = 0.01;

  }
  

}
