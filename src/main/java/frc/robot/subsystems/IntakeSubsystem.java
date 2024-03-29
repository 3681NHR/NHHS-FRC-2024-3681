// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

   private VictorSPX m_intakeBottom = new VictorSPX(Constants.INTAKE.BOTTOM_MOTOR_ID);
   private VictorSPX m_intakeTop    = new VictorSPX(Constants.INTAKE.TOP_MOTOR_ID   );
   private CANSparkMax m_rotate       = new CANSparkMax(Constants.INTAKE_SWING.MOTOR_ID, MotorType.kBrushless);

   private DigitalInput holdSwitch = new DigitalInput(Constants.INTAKE.DETECTOR_DIO_PIN);

   private double pidOut = 0.0;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.IDLE;

  private DutyCycleEncoder intakeSwingEncoder = new DutyCycleEncoder(Constants.INTAKE_SWING.ENCODER_DIO_PIN);

  private Measure<Angle> position;
  private boolean holding;
  private boolean switchEnabled = true;

  private Measure<Angle> downPos;
  private Measure<Angle> upPos;

  private ProfiledPIDController swingPID = new ProfiledPIDController(
  Constants.INTAKE_SWING.P_GAIN,
  Constants.INTAKE_SWING.I_GAIN,
  Constants.INTAKE_SWING.D_GAIN,
  new Constraints(10, 20)
  );
  //pids are for nerds like me
  private Measure<Angle> selectedAngle;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {

   //set motor idle modes
  
  }

  public double getAngleDeg(){
    return position.in(Degree);
  }
  public double getSelectedAngleDeg(){
    return selectedAngle.in(Degree);
  }
  
  public void setspeed(double val){
    m_intakeBottom.set(ControlMode.PercentOutput, val);
    m_intakeTop.set(ControlMode.PercentOutput, val);
  }

  public double getbottomspeed(){return m_intakeBottom.getMotorOutputPercent();}
  public double gettopspeed(){return m_intakeTop.getMotorOutputPercent();}


  public boolean isHolding(){
    if(switchEnabled){
      return holding;
    } else {
      return true;
    }
  }

  public String getSwingState(){return swingState.toString();}
  public String getState() {return state.toString();}
  public boolean getSwitchEnabled(){return switchEnabled;}
  public void setSwitchEnabled(boolean enabled){switchEnabled = enabled;}
 
  @Override
  public void periodic() {

    if(intakeSwingEncoder.getDistance() < Constants.INTAKE_SWING.PID_SWITCH.in(Radian)){
      upPos  = (MutableMeasure<Angle>) Constants.INTAKE_SWING.UP_POSITION.minus(Rotation.of(1));
      downPos = (MutableMeasure<Angle>) Constants.INTAKE_SWING.DOWN_POSITION.minus(Rotation.of(1));

      m_rotate.setInverted(true);
    } else {
      upPos = (MutableMeasure<Angle>)Constants.INTAKE_SWING.UP_POSITION;
      downPos = (MutableMeasure<Angle>)Constants.INTAKE_SWING.DOWN_POSITION;

      m_rotate.setInverted(true);
    }
    position = (MutableMeasure<Angle>) Radian.of(intakeSwingEncoder.getDistance());

    holding = !holdSwitch.get();
      
   this.m_intakeBottom.setNeutralMode(NeutralMode.Brake);
   this.m_intakeTop   .setNeutralMode(NeutralMode.Brake);
   this.m_rotate      .setIdleMode(IdleMode.kBrake);

    if(swingState == IntakeSwingState.UP){
      selectedAngle = upPos;
    }
    if(swingState == IntakeSwingState.DOWN){
      selectedAngle = downPos;
      if(holding && switchEnabled){
        swingState = IntakeSwingState.UP;
      }
    }
    if(swingState == IntakeSwingState.DOWN_HOLDING){
      selectedAngle = downPos;
    }
    if(swingState == IntakeSwingState.IDLE){
      selectedAngle = position;
    }

    //pid controller
    pidOut = clamp(swingPID.calculate(position.in(Radian), selectedAngle.in(Radian)), -Constants.INTAKE_SWING.MAX_OUTPUT, Constants.INTAKE_SWING.MAX_OUTPUT);

    m_rotate.set(pidOut);


  if(state == IntakeState.INTAKE){
    if(!holding || !switchEnabled){
      m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_OUTPUT);
      m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_OUTPUT);
    } else {
      setIntake(IntakeState.IDLE);
    }
  }
  if(state == IntakeState.COMPRESS){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_OUTPUT);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_OUTPUT);
  }
  if(state == IntakeState.REVERSE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.REVERSE_OUTPUT);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.REVERSE_OUTPUT);    
  } 
  if(state == IntakeState.IDLE){
    m_intakeBottom.set(ControlMode.PercentOutput, 0);
    m_intakeTop   .set(ControlMode.PercentOutput, 0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("intake swing angle"      , this::getAngleDeg                    , null);
    builder.addDoubleProperty("intake swing selected"   , this::getSelectedAngleDeg            , null);
    builder.addDoubleProperty("intake swing output"     , m_rotate::get                        , null);
    builder.addDoubleProperty("intake top roller out"   , m_intakeTop::getMotorOutputPercent   , null);
    builder.addDoubleProperty("intake bottom roller out", m_intakeBottom::getMotorOutputPercent, null);
    builder.addStringProperty("intake swing state"      , this::getSwingState                  , null);
    builder.addStringProperty("intake state"            , this::getState                       , null);
    builder.addBooleanProperty("intake switch"          , this::isHolding                      , null);
    builder.addBooleanProperty("intake switch enabled"  , this::getSwitchEnabled               , this::setSwitchEnabled);
  }
}