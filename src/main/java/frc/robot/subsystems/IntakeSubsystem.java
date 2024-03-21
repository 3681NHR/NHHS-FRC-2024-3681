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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double position = intakeSwingEncoder.getDistance();
  private boolean holding;
  private boolean switchEnabled = true;

  private double downPos;
  private double upPos;

  private ProfiledPIDController swingPID = new ProfiledPIDController(
  Constants.INTAKE_SWING.P_GAIN,
  Constants.INTAKE_SWING.I_GAIN,
  Constants.INTAKE_SWING.D_GAIN,
  new Constraints(10, 20)
  );
  //pids are for nerds like me
  private double selectedPosition;

  /** Creates a new Subsystem. */
  public IntakeSubsystem() {

   //set motor idle modes
  
    SmartDashboard.putBoolean("intake sensor enabled", switchEnabled);

    SmartDashboard.putNumber ("intake pid P gain", swingPID.getP());
    SmartDashboard.putNumber ("intake pid I gain", swingPID.getI());
    SmartDashboard.putNumber ("intake pid D gain", swingPID.getD()); 

    //m_rotate.setInverted(true);
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
    if(Math.abs(selectedPosition - position) <= Constants.INTAKE_SWING.POS_AE){
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
  
  public Command toggleSwingCommand() {
    return runOnce(
      () -> {
      if(swingState == IntakeSwingState.UP){
        if(!holding)
        {
          swingState = IntakeSwingState.DOWN;
        } else {
          swingState = IntakeSwingState.DOWN_HOLDING;
        }
      } else {
        swingState = IntakeSwingState.UP;
      }
      });
    }
    
    public Command toggleIntakeCommand() {
    return runOnce(
    () -> {
    if(state == IntakeState.INTAKE){
       state = IntakeState.IDLE;
     } else{
        state = IntakeState.INTAKE; 
      } 
    });
  
  }

  public Command runintakeCommand() {
    return runOnce(
    () -> {
    if(holding){
        setIntake(IntakeState.COMPRESS);
     } else{
        setIntake(IntakeState.INTAKE);
      } 
    });
  }

  public Command toggleReverseCommand() {
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
 
  @Override
  public void periodic() {

    if(intakeSwingEncoder.getDistance() < Constants.INTAKE_SWING.PID_SWITCH){
      upPos = Constants.INTAKE_SWING.UP_POSITION - 1;
      downPos = Constants.INTAKE_SWING.DOWN_POSITION - 1;

      m_rotate.setInverted(true);
    } else {
      upPos = Constants.INTAKE_SWING.UP_POSITION;
      downPos = Constants.INTAKE_SWING.DOWN_POSITION;

      m_rotate.setInverted(true);
    }

    switchEnabled = SmartDashboard.getBoolean("intake sensor enabled", true);
    
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
    SmartDashboard.putBoolean("IntakeIsHolding", holding);

    swingPID.setP(SmartDashboard.getNumber("intake pid P gain", Constants.INTAKE_SWING.P_GAIN));
    swingPID.setI(SmartDashboard.getNumber("intake pid I gain", Constants.INTAKE_SWING.I_GAIN));
    swingPID.setD(SmartDashboard.getNumber("intake pid D gain", Constants.INTAKE_SWING.D_GAIN));

    if(swingState == IntakeSwingState.UP){
      selectedPosition = upPos;
    }
    if(swingState == IntakeSwingState.DOWN){
      selectedPosition = downPos;
      if(holding && switchEnabled){
        swingState = IntakeSwingState.UP;
      }
    }
    if(swingState == IntakeSwingState.DOWN_HOLDING){
      selectedPosition = downPos;
    }
    if(swingState == IntakeSwingState.IDLE){
      selectedPosition = position;
    }

    //pid controller
    pidOut = clamp(swingPID.calculate(position, selectedPosition), -Constants.INTAKE_SWING.SPEED, Constants.INTAKE_SWING.SPEED);

    m_rotate.set(pidOut);


  if(state == IntakeState.INTAKE && (!holding || !switchEnabled)){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.SPEED);
  }
  if(state == IntakeState.COMPRESS){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.SPEED);
  }
  if(state == IntakeState.REVERSE){
    m_intakeBottom.set(ControlMode.PercentOutput, Constants.INTAKE.REVERSE_SPEED);
    m_intakeTop   .set(ControlMode.PercentOutput, Constants.INTAKE.REVERSE_SPEED);    
  } 
  if(state == IntakeState.IDLE){
    m_intakeBottom.set(ControlMode.PercentOutput, 0);
    m_intakeTop   .set(ControlMode.PercentOutput, 0);
  }
}
  
  private double clamp(double val, double min, double max)  {
    return Math.max(min, Math.min(max,val));
  }
}