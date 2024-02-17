// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;
import frc.robot.enums.IdleState;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.SparkWrapper;

public class DriveSubsystem extends SubsystemBase {

  private SparkWrapper m_back_left;
  private SparkWrapper m_back_right;
  private SparkWrapper m_front_left;
  private SparkWrapper m_front_right;

  double forward;
  double right; 
  double rotate;

  private Drive drive;

  private XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

  // Locations of the wheels relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); // TODO: Measure our robot and update these values
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381); // TODO: Measure our robot and update these values
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);   // TODO: Measure our robot and update these values
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);   // TODO: Measure our robot and update these values

  // Creating my kinematics object using the wheel locations.
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  // private final RelativeEncoder m_front_left_encoder = m_front_left.getEncoder();
  // private final RelativeEncoder m_front_right_encoder = m_front_right.getEncoder();
  // private final RelativeEncoder m_back_left_encoder = m_back_left.getEncoder();
  // private final RelativeEncoder m_back_right_encoder = m_back_right.getEncoder();

  // TODO: Right now, keeping initialization in the constructor; might want to move it to declaration time. 
  private RelativeEncoder m_front_left_encoder; 
  private RelativeEncoder m_front_right_encoder; 
  private RelativeEncoder m_back_left_encoder; 
  private RelativeEncoder m_back_right_encoder; 

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Creating my odometry object from the kinematics object and the initial wheel positions.
  // Here, our starting pose is 5 meters along the long end of the field and in the
  // center of the field along the short end, facing the opposing alliance wall.
  //
  // FROM DOCUMENTATION
  // MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
  //   m_kinematics,
  //   m_gyro.getRotation2d(),
  //   new MecanumDriveWheelPositions(
  //     m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
  //     m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
  //   ),
  //   new Pose2d(5.0, 13.5, new Rotation2d())
  // );


  MecanumDriveOdometry m_odometry; // = new MecanumDriveOdometry(
  //   m_kinematics,
  //   m_gyro.getRotation2d(),
  //   new MecanumDriveWheelPositions( // TODO: Check order of arguments
  //     m_front_left_encoder.getPosition(), m_front_right_encoder.getPosition(),
  //     m_back_left_encoder.getPosition(), m_back_right_encoder.getPosition()
  //   ),
  //   new Pose2d(5.0, 13.5, new Rotation2d()) // TODO: Check/update starting pose
  // );

  private Pose2d m_pose = new Pose2d(5.0, 13.5, new Rotation2d()); // TODO: Right now, using starting pose from creating m_odometry

  /** Creates a new Subsystem. */
  public DriveSubsystem() {
    // Initialize Motors
   this.m_back_left   = new SparkWrapper(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
   this.m_back_right  = new SparkWrapper(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
   this.m_front_left  = new SparkWrapper(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
   this.m_front_right = new SparkWrapper(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
  
   this.m_back_left  .setIdleMode(IdleState.BRAKE);
   this.m_back_right .setIdleMode(IdleState.BRAKE);
   this.m_front_left .setIdleMode(IdleState.BRAKE);
   this.m_front_right.setIdleMode(IdleState.BRAKE);

   // Initialize Encoders
   m_front_left_encoder = m_front_left.getEncoder();
   m_front_right_encoder = m_front_right.getEncoder();
   m_back_left_encoder = m_back_left.getEncoder();
   m_back_right_encoder = m_back_right.getEncoder();

   // Initialize Odometry
   m_odometry = new MecanumDriveOdometry(
    m_kinematics,
    m_gyro.getRotation2d(),
    new MecanumDriveWheelPositions( // TODO: Check order of arguments
      m_front_left_encoder.getPosition(), m_front_right_encoder.getPosition(),
      m_back_left_encoder.getPosition(), m_back_right_encoder.getPosition()
    ),
    new Pose2d(5.0, 13.5, new Rotation2d()) // TODO: Check/update starting pose
  );

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);
  }
  
  public Command Drive() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          drive.driveCartesian(rotate, right, forward);
        });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    forward = limit(Constants.DRIVE_INPUT_LIMITER, deadzone(-m_driverController.getLeftY(),  Constants.DRIVE_INPUT_DEADZONE));
    right   = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getLeftX() , Constants.DRIVE_INPUT_DEADZONE));
    rotate  = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getRightX(), Constants.DRIVE_INPUT_DEADZONE));
    
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("right"  ,   right);
    SmartDashboard.putNumber("rotate" ,  rotate);

    // Update the odometry
    // Get my wheel positions
    var wheelPositions = new MecanumDriveWheelPositions(
      m_front_left_encoder.getPosition(),  // TODO: This is rotations; need to setPositionConversionFactor() and/or convert to Meters
      m_front_right_encoder.getPosition(), 
      m_back_left_encoder.getPosition(), 
      m_back_right_encoder.getPosition());

    // Get the rotation of the robot from the gyro.
    var gyroAngle = m_gyro.getRotation2d();

    // Update the pose
    m_pose = m_odometry.update(gyroAngle, wheelPositions);
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_front_left_encoder.setPosition(0.0);
    m_front_right_encoder.setPosition(0.0);
    m_back_left_encoder.setPosition(0.0);
    m_back_right_encoder.setPosition(0.0);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), 
        new MecanumDriveWheelPositions(
          m_front_left_encoder.getPosition(),  // TODO: This is rotations; need to setPositionConversionFactor() and/or convert to Meters 
          m_front_right_encoder.getPosition(), 
          m_back_left_encoder.getPosition(), 
          m_back_right_encoder.getPosition()
        ), 
        pose); 
  }


}
