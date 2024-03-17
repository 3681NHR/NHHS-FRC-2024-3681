// done
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Drive;
import frc.robot.enums.DriveMode;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_back_left   = new CANSparkMax(Constants.DRIVE_BACK_LEFT_MOTOR_ID,   MotorType.kBrushless);
  private CANSparkMax m_back_right  = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_left  = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_MOTOR_ID,  MotorType.kBrushless);
  private CANSparkMax m_front_right = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_MOTOR_ID, MotorType.kBrushless);

  private RelativeEncoder m_back_left_encoder  ;
  private RelativeEncoder m_back_right_encoder ;
  private RelativeEncoder m_front_left_encoder ;
  private RelativeEncoder m_front_right_encoder;

  private double forward;
  private double right; 
  private double rotate;

  private Rotation2d angle = new Rotation2d();
  private boolean FOD = true;
  private ADIS16448_IMU gyro = new ADIS16448_IMU();
  private double offset = 0.0;

  private boolean squaringEnabled = true;
  private boolean modeChangeEnabled = false;

  private DriveMode mode;
  private Drive drive;

  private XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);//change to DRIVER_CONTROLLER_PORT to use duel controller

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

//region sysid
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_back_left  .setVoltage(volts.in(Volts));
                m_back_right .setVoltage(volts.in(Volts));
                m_front_left .setVoltage(volts.in(Volts));
                m_front_right.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the motors
                log.motor("drive-back-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_back_left.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_back_left_encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_back_left_encoder.getVelocity(), MetersPerSecond));
                
                log.motor("drive-back-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_back_right.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_back_right_encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_back_right_encoder.getVelocity(), MetersPerSecond));
                
                log.motor("drive-front-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_front_left.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_front_left_encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_front_left_encoder.getVelocity(), MetersPerSecond));
                
                log.motor("drive-front-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_front_right.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_front_right_encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_front_right_encoder.getVelocity(), MetersPerSecond));
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
//endregion
  
/** Creates a new Subsystem. */
  public DriveSubsystem() {
    System.out.println("drive initalized");

    setMotorIdleMode();//idfk why i need this but it works

   this.m_back_right .setInverted(true);
   this.m_front_right.setInverted(true);

   drive = new Drive(m_front_left, m_back_left, m_front_right, m_back_right);

   m_back_left_encoder   = m_back_left.getEncoder(); 
   m_back_right_encoder  = m_back_right.getEncoder(); 
   m_front_left_encoder  = m_front_left.getEncoder(); 
   m_front_right_encoder = m_front_right.getEncoder(); 

   m_back_left_encoder.setPositionConversionFactor(Constants.DRIVE_DISANCE_PER_PULSE); 
   m_back_right_encoder.setPositionConversionFactor(Constants.DRIVE_DISANCE_PER_PULSE); 
   m_front_left_encoder.setPositionConversionFactor(Constants.DRIVE_DISANCE_PER_PULSE); 
   m_front_right_encoder.setPositionConversionFactor(Constants.DRIVE_DISANCE_PER_PULSE); 

    SmartDashboard.putBoolean("field oriented driving", FOD);
    SmartDashboard.putBoolean("input squaring", squaringEnabled);
    SmartDashboard.putBoolean("input sensitivity buttons", modeChangeEnabled);
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

    if(FOD){
      angle = new Rotation2d(Math.toRadians(gyro.getGyroAngleZ()+offset));
    } else {
      angle = new Rotation2d(0);
    }
    
    drive.driveCartesian(forward, right, -rotate, angle);

    SmartDashboard.putNumber("gyro", angle.getDegrees());
    SmartDashboard.putNumber("forward"   , forward        );
    SmartDashboard.putNumber("right"     , right          );
    SmartDashboard.putNumber("rotate"    , rotate         );
    SmartDashboard.putBoolean("field oriented driving", FOD);
    SmartDashboard.putBoolean("input squaring", squaringEnabled);
    SmartDashboard.putBoolean("input sensitivity buttons", modeChangeEnabled);
  }
  public void teleopPeriodic(){
    if(squaringEnabled){
      forward = Math.pow(deadzone(-m_driverController.getLeftY() , Constants.DRIVE_INPUT_DEADZONE), 3);
      right   = Math.pow(deadzone(m_driverController.getLeftX() , Constants.DRIVE_INPUT_DEADZONE) , 3);
      rotate  = Math.pow(deadzone(m_driverController.getRightX(), Constants.DRIVE_INPUT_DEADZONE) , 3);
    } //square the value, then reaply the sign using the normalize function
    else 
    {
      forward = deadzone(-m_driverController.getLeftY() , Constants.DRIVE_INPUT_DEADZONE);
      right   = deadzone( m_driverController.getLeftX() , Constants.DRIVE_INPUT_DEADZONE);
      rotate  = deadzone( m_driverController.getRightX(), Constants.DRIVE_INPUT_DEADZONE);
    }
    if(m_driverController.getRightBumper()){//set mode(only does anything if mode change is enabled)
      mode = DriveMode.FAST;
    } else if(m_driverController.getLeftBumper()){
      mode = DriveMode.SLOW;
    } else {
      mode = DriveMode.MEDIUM;
    }
    if(modeChangeEnabled){//remap if mode change is enabled
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
  }

  public void setAutoMotion(double f, double r, double rot){
    this.forward = f;
    this.right   = r;
    this.rotate  = rot;
  }
  /**
   * sets Field-Oriented-Driving(FOD)
   * @param enabled - set FOD enabled
   */
  public Command setFOD(boolean enabled){
    return runOnce(() -> {
      this.FOD = enabled;
    });
  }
  public Command toggleFOD(){
    return runOnce(() -> {
      this.FOD = !this.FOD;
    });
  }
  /** toggle input squaring 
   * <p>{@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs}
  */
  public Command togglesquaring(){
    return runOnce(() -> {
      this.squaringEnabled = !this.squaringEnabled;
    });
  }
  /** toggle multiple sensitivities
   * <p> when on, input mode is medium, causing controlable, but slower speeds by default. 
   * bumpers can be used to change to high and low sensitivity modes
   * <p> when off, input is a full sensitivity the whole time, recomended if using input squaring
   */
  public Command toggleModeChanging(){
    return runOnce(() -> {
      this.modeChangeEnabled = !this.modeChangeEnabled;
    });
  }
  /** sets current headding to absolute forward when using FOD */
  public Command zero(){
    return runOnce(() -> {
      this.offset = -gyro.getGyroAngleZ();
    });
  }
  /**
   * zeros with current angle set to offset
   * @param offset
   */
  public Command zero(double offset){
    return runOnce(() -> {
      this.offset = -gyro.getGyroAngleZ() + offset;
    });
  }
  /** recalibrate the gyro, requires 10 seconds of absolutly no motion */
  public Command resetGyro(){
    return runOnce(() -> {
      gyro.reset();
    });
  }

  public Command sysidQuasistatic(SysIdRoutine.Direction dir){
    return m_sysIdRoutine.quasistatic(dir);
  }
  public Command sysidDynamic(SysIdRoutine.Direction dir){
    return m_sysIdRoutine.dynamic(dir);
  }

  private double deadzone(double value, double zone)
  {
    double x = value;

    if(Math.abs(x)<zone){
      x=0;
    }
    return x;
  }
  
  private double remap(double input, double start_top, double start_bottom, double end_top, double end_bottom){
    return end_bottom + ((end_top-end_bottom)*((input-start_bottom) / (start_top-start_bottom)));
  }
  @SuppressWarnings("unused")
  private double normalize(double input){//only needed for squaring with even exponents
    return input/Math.abs(input);
  }
}
