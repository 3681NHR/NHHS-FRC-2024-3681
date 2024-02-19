// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import java.util.List;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private Joystick joystick = new Joystick(0);

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

  private Pose2d m_pose = new Pose2d(1.0, 1.5, new Rotation2d()); // TODO: Right now, using starting pose from creating m_odometry

  // Simulated Hardware Definitions
  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim m_front_left_encoder_sim; // = new EncoderSim(m_leftEncoder);
  private EncoderSim m_front_right_encoder_sim; 
  private EncoderSim m_back_left_encoder_sim; 
  private EncoderSim m_back_right_encoder_sim; 

  private REVPhysicsSim physicsSim;
  private Trajectory m_trajectory; 
  
  private Field2d field = null; 

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
    new Pose2d(1.0, 1.5, new Rotation2d()) // TODO: Check/update starting pose
  );

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

   // Initialize Simulated Hardware
  //  m_front_left_encoder_sim = new EncoderSim();
  //  m_front_right_encoder = m_front_right.getEncoder();
  //  m_back_left_encoder = m_back_left.getEncoder();
  //  m_back_right_encoder = m_back_right.getEncoder();

   // TODO: If "Simulation"
    // this.physicsSim = REVPhysicsSim.getInstance();
    // this.physicsSim.addSparkMax(m_front_left.getCanSparkMax(), DCMotor.getNeo550(1));
    // this.physicsSim.addSparkMax(m_front_right.getCanSparkMax(), DCMotor.getNeo550(1));
    // this.physicsSim.addSparkMax(m_back_left.getCanSparkMax(), DCMotor.getNeo550(1));
    // this.physicsSim.addSparkMax(m_back_right.getCanSparkMax(), DCMotor.getNeo550(1));

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
  }
  
  public Command Drive() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          System.out.println(rotate + ", " + right + ", " + forward);

          drive.driveCartesian(rotate, right, forward); // TODO: These value don't map to what they seem. 
          SmartDashboard.putData("Drive Command", this);
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

  // Hacking, just to see if the simulator can work
  public CANSparkMax getFLCanSparkMax() {
    return this.m_front_left.getCanSparkMax();
  }

  public CANSparkMax getFRCanSparkMax() {
    return this.m_front_right.getCanSparkMax();
  }

  public CANSparkMax getBLCanSparkMax() {
    return this.m_back_left.getCanSparkMax();
  }

  public CANSparkMax getBRCanSparkMax() {
    return this.m_back_right.getCanSparkMax();
  }

  public void setField(Field2d field) {
    this.field = field; 
  }
  

  @Override
  public void simulationPeriodic()  {
    // System.out.println("Doing a simulation thing in the subsystem");
    // System.out.println("driverController: " + m_driverController.getLeftY());
    // System.out.println("driverController: " + m_driverController.getRawAxis(1));
    // System.out.println("Joystick: " + joystick.getRawAxis(1));
    REVPhysicsSim.getInstance().run();

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
    // Do this in either robot periodic or subsystem periodic
    if (field != null) {
      // field.setRobotPose(m_odometry.getPoseMeters());
      field.setRobotPose(this.m_pose);
      System.out.println(this.m_pose);
    }

  }

}
