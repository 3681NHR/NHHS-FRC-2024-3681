// done
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_left  = new CANSparkMax(Constants.LAUNCHER.LEFT_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_right = new CANSparkMax(Constants.LAUNCHER.RIGHT_MOTOR_ID, MotorType.kBrushless);

  private Measure<Velocity<Angle>> m_left_target = RPM.of(0);
  private Measure<Velocity<Angle>> m_right_target =RPM.of(0);

  private PIDController m_left_PID  = new PIDController(0, 0, 0);
  private PIDController m_right_PID = new PIDController(0, 0, 0);

  public LauncherSubsystem() {
    this.m_left.setIdleMode(IdleMode.kCoast);
    this.m_right.setIdleMode(IdleMode.kCoast);
  }
  
  public Measure<Velocity<Angle>> getSpeed(boolean right){
    if(right){
      return RPM.of(m_right.getEncoder().getVelocity());
    } else {
      return RPM.of(m_left.getEncoder().getVelocity());
    }
  }
  public double getLeftSpeed(){
    return m_left.getEncoder().getVelocity();
  }
  public double getRightSpeed(){
    return m_right.getEncoder().getVelocity();
  }

  public void setSpeed(Measure<Velocity<Angle>> speed){
    setSpeed(speed, speed);
  }

  public void stopMotors(){setSpeed(RPM.of(0));}

  public void setSpeed(Measure<Velocity<Angle>> right, Measure<Velocity<Angle>> left){
    m_left_target = left;
    m_right_target = right;
  }

  @Override
  public void periodic() {

    updateMotors();

    SmartDashboard.putNumber("L launcher speed", m_left.getEncoder().getVelocity());
    SmartDashboard.putNumber("R launcher speed", m_right.getEncoder().getVelocity());
  }
  private void updateMotors(){
    m_left  .set(m_left_PID.calculate(getSpeed(false).in(RPM), m_left_target.in(RPM)));
    m_right .set(m_right_PID.calculate(getSpeed(true).in(RPM), m_right_target.in(RPM)));
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty ("launcher right output"      , m_right::get            , null);
    builder.addDoubleProperty ("launcher right speed"       , this::getRightSpeed     , null);
    builder.addDoubleProperty ("launcher right target speed", m_right_PID::getSetpoint, null);
    builder.addDoubleProperty ("launcher left output"       , m_right::get            , null);
    builder.addDoubleProperty ("launcher left speed"        , this::getLeftSpeed      , null);
    builder.addDoubleProperty ("launcher left target speed" , m_left_PID::getSetpoint , null);
  }
}

