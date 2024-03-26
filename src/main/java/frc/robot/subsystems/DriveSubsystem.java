// done
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

import frc.utils.Vector;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Drive;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left  ;
  private CANSparkMax m_back_right ;
  private CANSparkMax m_front_left ;
  private CANSparkMax m_front_right;

  private ProfiledPIDController rotatePID = new ProfiledPIDController(
    Constants.DRIVE.ROTATE_P_GAIN,  
    Constants.DRIVE.ROTATE_I_GAIN,  
    Constants.DRIVE.ROTATE_D_GAIN,  
    null
    );

  private Measure<Angle> angle = Radian.of(0);
  private ADIS16448_IMU gyro = new ADIS16448_IMU();
  private Measure<Angle> offset = Degree.of(0.0);

  private Drive drive;

  /** Creates a new Subsystem. */
  public DriveSubsystem() {

    this.m_back_left   = new CANSparkMax(Constants.DRIVE.BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
    this.m_back_right  = new CANSparkMax(Constants.DRIVE.BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
    this.m_front_left  = new CANSparkMax(Constants.DRIVE.FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
    this.m_front_right = new CANSparkMax(Constants.DRIVE.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);

    this.m_back_right .setInverted(true);
    this.m_front_right.setInverted(true);

    m_back_left  .setIdleMode(IdleMode.kBrake);
    m_back_right .setIdleMode(IdleMode.kBrake);
    m_front_left .setIdleMode(IdleMode.kBrake);
    m_front_right.setIdleMode(IdleMode.kBrake);

    drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);
  }

  @Override
  public void periodic() {
      angle = Degree.of(gyro.getGyroAngleZ()+offset.in(Degree)); 
  }

  public void drive(Vector leftStick, Vector rightStick, boolean FOD, boolean altRotate){
    if(altRotate){
      driveMain(leftStick, DegreesPerSecond.of(rightStick.x()), FOD);
    } else {
      driveAltRotate(leftStick, Degree.of(rightStick.angle()), FOD);
    }
  }

  public void driveAltRotate(Vector leftStick, Measure<Angle> rotation, boolean FOD){
    double rotateOut = rotatePID.calculate(angle.in(Degree), rotation.in(Rotation));
    if(FOD){
      drive.drivePolar(leftStick, rotateOut, new Rotation2d(angle.in(Radian)));
    } else {
      drive.drivePolar(leftStick, rotateOut);
    }
  }

  public void driveMain(Vector leftStick, Measure<Velocity<Angle>> rotate, boolean FOD){
    if(FOD){
      drive.drivePolar(leftStick, -rotate.in(DegreesPerSecond), new Rotation2d(angle.in(Radian)));
    } else {
      drive.drivePolar(leftStick, -rotate.in(DegreesPerSecond));
    }
  }

  public void stop(){
    drive.stopMotor();
  }

  public void sendTelemetry(boolean useLogs){
    SmartDashboard.putNumber("gyro", angle.in(Degree));
    //SmartDashboard.putBoolean("field oriented driving", FOD);
    //SmartDashboard.putBoolean("input squaring", squaringEnabled);
    //SmartDashboard.putBoolean("input sensitivity buttons", modeChangeEnabled);
    SmartDashboard.putNumber("gyro offset", offset.in(Degree));

    if(useLogs){

    }
  }
}
