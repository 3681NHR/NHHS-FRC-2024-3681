// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax.IdleMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.enums.IntakeState;
import frc.robot.enums.IntakeSwingState;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

   private VictorSPX m_intakeBottom = new VictorSPX(Constants.INTAKE_BOTTOM_MOTOR_ID);
   private VictorSPX m_intakeTop    = new VictorSPX(Constants.INTAKE_TOP_MOTOR_ID   );
   private VictorSPX m_rotate       = new VictorSPX(Constants.INTAKE_SWING_MOTOR_ID );

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
  //pids are for nerds like me
  private double selectedPosition;

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
    
    position = intakeSwingEncoder.getDistance();

    holding = !holdSwitch.get();
      
   this.m_intakeBottom.setNeutralMode(NeutralMode.Brake);
   this.m_intakeTop   .setNeutralMode(NeutralMode.Brake);
   this.m_rotate      .setNeutralMode(NeutralMode.Brake);

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

    m_rotate.set(ControlMode.PercentOutput, pidOut);


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

