// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;

public class IntakeSubsystem extends SubsystemBase {

   private VictorSPX m_intakeBottom = new VictorSPX(Constants.INTAKE_BOTTOM_MOTOR_ID);
   private VictorSPX m_intakeTop    = new VictorSPX(Constants.INTAKE_TOP_MOTOR_ID   );
   private CANSparkMax m_rotate       = new CANSparkMax(Constants.INTAKE_SWING_MOTOR_ID, MotorType.kBrushless);

   private double intakeSwingEncoderRate = 0;
   private double intakeSwingEncoderLastFrame = 0;

   private DigitalInput holdSwitch = new DigitalInput(Constants.INTAKE_DETECTOR_DIO_PIN);

   private double pidOut = 0.0;
  
  private IntakeState state = IntakeState.IDLE;
  private IntakeSwingState swingState = IntakeSwingState.IDLE;

  private DutyCycleEncoder intakeSwingEncoder = new DutyCycleEncoder(Constants.INTAKE_SWING_ENCODER_DIO_PIN);

  private double position = intakeSwingEncoder.getDistance();
  private boolean holding;

  private PIDController swingPID = new PIDController(
  Constants.INTAKE_SWING_P_GAIN,
  Constants.INTAKE_SWING_I_GAIN,
  Constants.INTAKE_SWING_D_GAIN
  );

  private double selectedPosition;

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
                m_rotate.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the motors
                log.motor("intake swing")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_rotate.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(intakeSwingEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(intakeSwingEncoderRate, MetersPerSecond));
                
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
//endregion
  /** Creates a new Subsystem. */
  public IntakeSubsystem() {
   //set motor idle modes

   swingPID.setTolerance(Constants.INTAKE_SWING_POS_AE, Constants.INTAKE_SWING_PID_VELOCITY_TOLERANCE);
  }

  
  public double getPosition(boolean selectedPos){
    if(selectedPos){
      return selectedPosition;
    } else{
    return position;
    }
  }
  public void setPosition(IntakeSwingState s){
    swingState = s;
  }

  public boolean isAtSelectedPos(){
    if(Math.abs(selectedPosition - position) <= Constants.INTAKE_SWING_POS_AE){
      return true;
    } else {
      return false;
    }
  }

  public void setIntake(IntakeState state){
    this.state = state;
  }
  public Command setIntakeCommand(IntakeState s){
    return runOnce(() -> {
      state = s;
    }
    );
  }
  

  public Command toggleSwing() {
    return runOnce(
      () -> {
      if(swingState == IntakeSwingState.UP){
        swingState = IntakeSwingState.DOWN;
      } else{
        swingState = IntakeSwingState.UP;
      }
      });
    }
    
    public Command toggleIntake() {
    return runOnce(
    () -> {
    if(state == IntakeState.INTAKE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.INTAKE; 
      } 
    });
  
  }

    public Command toggleReverse() {
    return runOnce(
    () -> {
    if(state == IntakeState.REVERSE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.REVERSE;
      } 
    });
  }  
  public boolean isHolding(){
    return holding;
  }
    
  public Command sysidQuasistatic(SysIdRoutine.Direction dir){
    return m_sysIdRoutine.quasistatic(dir);
  }
  public Command sysidDynamic(SysIdRoutine.Direction dir){
    return m_sysIdRoutine.dynamic(dir);
  }

  @Override
  public void periodic() {

    intakeSwingEncoderRate = (intakeSwingEncoderLastFrame-intakeSwingEncoder.getDistance())*(20/1000);
    intakeSwingEncoderLastFrame = intakeSwingEncoder.getDistance();
    
    position = intakeSwingEncoder.getDistance();

    holding = !holdSwitch.get();
      
   this.m_intakeBottom.setNeutralMode(NeutralMode.Brake);
   this.m_intakeTop   .setNeutralMode(NeutralMode.Brake);
   this.m_rotate      .setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("intake swing selected pos" , selectedPosition                );
    SmartDashboard.putNumber("intake swing current pos"  , position);
    SmartDashboard.putString("intake swing state"        , swingState.toString()           );
    SmartDashboard.putString("intake state"              , state.toString()                );
    SmartDashboard.putNumber("intake swing PID value"    , pidOut                          );

    if(swingState == IntakeSwingState.UP){
      selectedPosition = Constants.INTAKE_SWING_UP_POSITION;
    }
    if(swingState == IntakeSwingState.DOWN){
      selectedPosition = Constants.INTAKE_SWING_DOWN_POSITION;
    }
    if(swingState == IntakeSwingState.IDLE){
      selectedPosition = position;
    }

    if(position < selectedPosition){
    pidOut = clamp(6 * (selectedPosition - position), -Constants.INTAKE_SWING_UP_SPEED,Constants.INTAKE_SWING_UP_SPEED);
    } else {
    pidOut = clamp(1.5 * (selectedPosition - position), -Constants.INTAKE_SWING_DOWN_SPEED,Constants.INTAKE_SWING_DOWN_SPEED);
    } 
    if(intakeSwingEncoder.getDistance() > 1){
      pidOut = clamp(1, -Constants.INTAKE_SWING_SPEED, Constants.INTAKE_SWING_SPEED);
    }
    //p controller
    //pidOut = clamp(swingPID.calculate(position,selectedPosition), -Constants.INTAKE_SWING_SPEED,Constants.INTAKE_SWING_SPEED);

    m_rotate.set(pidOut);


  if(state == IntakeState.INTAKE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }else if(state == IntakeState.REVERSE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE_REVERSE_SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE_REVERSE_SPEED);    
  } else{
    m_intakeBottom.set(ControlMode.PercentOutput, 0);
    m_intakeTop   .set(ControlMode.PercentOutput, 0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));
  }
}

