
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.enums.DriveMode;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left   = new CANSparkMax(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
  private CANSparkMax m_back_right  = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_left  = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_right = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);
 
  private RelativeEncoder m_back_left_encoder = m_back_left.getEncoder();
  private RelativeEncoder m_back_right_encoder = m_back_right.getEncoder();
  private RelativeEncoder m_front_left_encoder = m_front_left.getEncoder();
  private RelativeEncoder m_front_right_encoder = m_front_right.getEncoder(); 
  

  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    new Translation2d()
  , new Translation2d()
  , new Translation2d()
  , new Translation2d());//wheel positions 
  
  private double forward;
  private double right; 
  private double rotate;
  private double wheelMaxSpeed = 10000;//placeholder

  private double inputSpeedMultiplyer = 1;
  private double inputRotationSpeedMultiplyer = 0.5;

  ChassisSpeeds speeds = new ChassisSpeeds();

  private DriveMode mode;

  private XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);//change to DRIVER_CONTROLLER_PORT to use duel controller

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  /** Creates a new Subsystem. */
  public DriveSubsystem() {

    setMotorIdleMode();//idfk why i need this but it works
   
   this.m_back_right .setInverted(true);
   this.m_front_right.setInverted(true);

  }

  private void setMotorIdleMode(){
     //should be kOk if no error
   m_back_left  .setIdleMode(IdleMode.kBrake);
   m_back_right .setIdleMode(IdleMode.kBrake);
   m_front_left .setIdleMode(IdleMode.kBrake);
   m_front_right.setIdleMode(IdleMode.kBrake);

  }

  public MecanumDriveWheelSpeeds getCurrentSpeeds(){
    return new MecanumDriveWheelSpeeds(
      m_back_left_encoder.getVelocity(),
      m_back_right_encoder.getVelocity(),
      m_front_left_encoder.getVelocity(),
      m_front_right_encoder.getVelocity()
      );
 }
  public MecanumDriveWheelPositions getCurrentDistances(){
    return new MecanumDriveWheelPositions(
      m_back_left_encoder.getPosition(),
      m_back_right_encoder.getPosition(),
      m_front_left_encoder.getPosition(),
      m_front_right_encoder.getPosition());

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    speeds.vxMetersPerSecond     = forward * inputSpeedMultiplyer;
    speeds.vyMetersPerSecond     = -right  * inputSpeedMultiplyer;
    speeds.omegaRadiansPerSecond = -rotate * inputRotationSpeedMultiplyer;

    // Convert to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    wheelSpeeds.desaturate(wheelMaxSpeed);

    final double frontLeftFeedforward = m_feedforward.calculate(wheelSpeeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(wheelSpeeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(wheelSpeeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(wheelSpeeds.rearRightMetersPerSecond);

    final double frontLeftOutput =
        m_frontLeftPIDController.calculate(
            m_front_left.getEncoder().getVelocity(), wheelSpeeds.frontLeftMetersPerSecond);
    final double frontRightOutput =
        m_frontRightPIDController.calculate(
            m_front_right.getEncoder().getVelocity(), wheelSpeeds.frontRightMetersPerSecond);
    final double backLeftOutput =
        m_backLeftPIDController.calculate(
            m_back_left.getEncoder().getVelocity(), wheelSpeeds.rearLeftMetersPerSecond);
    final double backRightOutput =
        m_backRightPIDController.calculate(
            m_back_right.getEncoder().getVelocity(), wheelSpeeds.rearRightMetersPerSecond);


    // Get the individual wheel speeds
    m_front_left .setVoltage(frontLeftOutput  + frontLeftFeedforward );
    m_front_right.setVoltage(frontRightOutput + frontRightFeedforward);
    m_back_left  .setVoltage(backLeftOutput   + backLeftFeedforward  );
    m_back_right .setVoltage(backRightOutput  + backRightFeedforward );
    
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("right"  , right  );
    SmartDashboard.putNumber("rotate" , rotate );
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
