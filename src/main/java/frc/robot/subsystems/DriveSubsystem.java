// done
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;
import frc.robot.enums.DriveMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left   = new CANSparkMax(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
  private CANSparkMax m_back_right  = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_left  = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_right = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);

  private double forward;
  private double right; 
  private double rotate;

  private ADIS16448_IMU gyro = new ADIS16448_IMU();

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

   m_back_left  .setIdleMode(IdleMode.kBrake);
   m_back_right .setIdleMode(IdleMode.kBrake);
   m_front_left .setIdleMode(IdleMode.kBrake);
   m_front_right.setIdleMode(IdleMode.kBrake);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    drive.driveCartesian(forward, right, -rotate);

    SmartDashboard.putNumber("gyro", gyro.getGyroAngleZ());
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
