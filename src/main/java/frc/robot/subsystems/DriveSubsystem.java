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
import frc.robot.enums.DriveMode;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  private ADIS16448_IMU IMU = new ADIS16448_IMU();
  private Measure<Angle> offset = Degree.of(0.0);

  private DriveMode drivemode;
  private boolean modeChangeEnabled = false;
  
  private int squaringvalue;

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
    
    SmartDashboard.putData("drivetrain", drive);
    SmartDashboard.putData("PID/drivetrain rotation", rotatePID);
    SmartDashboard.putData("IMU", IMU);
  }

  @Override
  public void periodic() {

    sendTelemetry(false);

    angle = Degree.of(IMU.getGyroAngleZ()+offset.in(Degree)); 

  }
  public void setMode(DriveMode mode){drivemode = mode;}
  public void setInputSquaring(long squaring){
    if(squaring > 0){
      squaringvalue = (int) squaring;
    } else {
      squaringvalue = 1;
    }
  }
  public int getSquaring(){return squaringvalue;}
  public void setInputmodes(boolean enabled){modeChangeEnabled = enabled;}
  public boolean getInputModes(){return modeChangeEnabled;}

  public void zero(){offset = Degree.of(-IMU.getGyroAngleY());}
  public void zero(Measure<Angle> offset){
    this.offset = Degree.of(-IMU.getGyroAngleY()).plus(offset);
  }

  public void drive(Vector leftStick, Vector rightStick, boolean FOD, boolean altRotate){
    Vector prossesedLeftStick = leftStick;
    Vector prossesedRightStick = rightStick;

    prossesedLeftStick  = prossesedLeftStick .deadband(Constants.DRIVE.INPUT_DEADZONE);
    prossesedRightStick = prossesedRightStick.deadband(Constants.DRIVE.INPUT_DEADZONE);
  
    //uses math.pow to raise to pow, the reapply sign using signum, raise sgn to pow+1 to negate is when pow is odd, as odd pow does not need sgn
    prossesedLeftStick  = new Vector(Math.pow(prossesedLeftStick .x(), squaringvalue) * Math.pow(Math.signum(prossesedLeftStick .x()), squaringvalue+1), Math.pow(prossesedLeftStick .y(), squaringvalue) * Math.pow(Math.signum(prossesedLeftStick .y()), squaringvalue+1));
    prossesedRightStick = new Vector(Math.pow(prossesedRightStick.x(), squaringvalue) * Math.pow(Math.signum(prossesedRightStick.x()), squaringvalue+1), Math.pow(prossesedRightStick.y(), squaringvalue) * Math.pow(Math.signum(prossesedRightStick.y()), squaringvalue+1));

    if(modeChangeEnabled){
      if(drivemode == DriveMode.FAST){
        prossesedLeftStick = prossesedLeftStick.mult(Constants.DRIVE.FAST_SPEED_MAX_INPUT);
        prossesedRightStick = prossesedRightStick.mult(Constants.DRIVE.FAST_SPEED_MAX_INPUT);
      }
      if(drivemode == DriveMode.MEDIUM){
        prossesedLeftStick = prossesedLeftStick.mult(Constants.DRIVE.MEDIUM_SPEED_MAX_INPUT);
        prossesedRightStick = prossesedRightStick.mult(Constants.DRIVE.MEDIUM_SPEED_MAX_INPUT);
      }
      if(drivemode == DriveMode.SLOW){
        prossesedLeftStick = prossesedLeftStick.mult(Constants.DRIVE.SLOW_SPEED_MAX_INPUT);
        prossesedRightStick = prossesedRightStick.mult(Constants.DRIVE.SLOW_SPEED_MAX_INPUT);
      }
    }
    
    if(!altRotate){
      driveMain(prossesedLeftStick, DegreesPerSecond.of(prossesedRightStick.x()), FOD);
    } else {
      driveAltRotate(prossesedLeftStick, Degree.of(prossesedRightStick.angle()), FOD);
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

  public void stop(){drive.stopMotor();}

  public void sendTelemetry(boolean useLogs){
    if(useLogs){

    }
  }
  
  public double getAngle(){return angle.in(Degree);}
  public double getAngleRaw(){return IMU.getGyroAngleZ();}
  public double getAngleOffset(){return offset.in(Degree);}
  public void setAngleOffset(double offset){this.offset = Degree.of(offset);}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("input modes enabled", this::getInputModes, this::setInputmodes);
    builder.addIntegerProperty("squaring value", this::getSquaring, this::setInputSquaring);

    builder.addDoubleProperty("gyro prossesed", this::getAngle, null);
    builder.addDoubleProperty("gyro raw"      , this::getAngleRaw, null);
    builder.addDoubleProperty("gyro offset"   , this::getAngleOffset, this::setAngleOffset);

  }
}
