
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;
import frc.robot.enums.DriveMode;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left   = new CANSparkMax(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
  private CANSparkMax m_back_right  = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_left  = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_right = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(new Translation2d()
  , new Translation2d()
  , new Translation2d()
  , new Translation2d());
  private MecanumDriveOdometry odometry = new MecanumDriveOdometry(
    m_kinematics,
    m_gyro.getRotation2d(),
    new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getDistance(),m_frontRightEncoder.getDistance(),
      m_backLeftEncoder.getDistance(),m_backRightEncoder.getDistance()
    ),
    // Here, our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing the opposing alliance wall
    new Pose2d(5.0,13.5,new Rotation2d())
  );
  private double forward;
  private double right; 
  private double rotate;

  private DriveMode mode;

  private Drive drive;

  private XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);//change to DRIVER_CONTROLLER_PORT to use duel controller

  /** Creates a new Subsystem. */
  public DriveSubsystem() {
    System.out.println("drive initalized");

    setMotorIdleMode();//idfk why i need this but it works
   
   this.m_back_right .setInverted(true);
   this.m_front_right.setInverted(true);

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

  }

  private void setMotorIdleMode(){
     //should be kOk if no error
   m_back_left  .setIdleMode(IdleMode.kBrake);
   m_back_right .setIdleMode(IdleMode.kBrake);
   m_front_left .setIdleMode(IdleMode.kBrake);
   m_front_right.setIdleMode(IdleMode.kBrake);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Locations of the wheels relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0, 0);
    Translation2d m_frontRightLocation = new Translation2d(0, 0);
    Translation2d m_backLeftLocation = new Translation2d(0, 0);
    Translation2d m_backRightLocation = new Translation2d(0, 0);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;



    
    
    SmartDashboard.putNumber("forward"   , forward        );
    SmartDashboard.putNumber("right"     , right          );
    SmartDashboard.putNumber("rotate"    , rotate         );
  }
  public void teleopPeriodic(){
    forward = limit(Constants.DRIVE_INPUT_LIMITER, deadzone(-m_driverController.getLeftY(),  Constants.DRIVE_INPUT_DEADZONE));
    right   = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getLeftX() , Constants.DRIVE_INPUT_DEADZONE));
    rotate  = limit(Constants.DRIVE_INPUT_LIMITER, deadzone( m_driverController.getRightX(), Constants.DRIVE_INPUT_DEADZONE));

    if(m_driverController.getRightBumper()){
      mode = DriveMode.FAST;
    } else if(m_driverController.getLeftBumper()){
      mode = DriveMode.SLOW;
    } else {
      mode = DriveMode.MEDIUM;
    }

    if(mode == DriveMode.MEDIUM){
      forward = remap(forward, -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT, Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT);
      right   = remap(right  , -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT, Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT);
      rotate  = remap(rotate , -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT, Constants.DRIVE_MEDIUM_SPEED_MAX_INPUT);
    } else if (mode == DriveMode.SLOW){
      forward = remap(forward, -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_SLOW_SPEED_MAX_INPUT, Constants.DRIVE_SLOW_SPEED_MAX_INPUT);
      right   = remap(right  , -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_SLOW_SPEED_MAX_INPUT, Constants.DRIVE_SLOW_SPEED_MAX_INPUT);
      rotate  = remap(rotate , -Constants.DRIVE_INPUT_LIMITER, Constants.DRIVE_INPUT_LIMITER, -Constants.DRIVE_SLOW_SPEED_MAX_INPUT, Constants.DRIVE_SLOW_SPEED_MAX_INPUT);
    }
  }

  public void setAutoMotion(double f, double r, double rot){
    this.forward = f;
    this.right   = r;
    this.rotate  = rot;
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
  private double remap(double input, double start_top, double start_bottom, double end_top, double end_bottom){
    return end_bottom + ((end_top-end_bottom)*((input-start_bottom) / (start_top-start_bottom)));
    
  }

}
