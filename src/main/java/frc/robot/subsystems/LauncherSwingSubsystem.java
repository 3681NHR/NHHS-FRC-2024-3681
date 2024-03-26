// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degree;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSwingSubsystem extends SubsystemBase {

  private CANSparkMax swingMotor  = new CANSparkMax(Constants.LAUNCHER_SWING.MOTOR_ID, MotorType.kBrushless);

  private Measure<Voltage> PIDOut;
  private Measure<Voltage> feedforwardOut;

  private DutyCycleEncoder swingEncoder = new DutyCycleEncoder(Constants.LAUNCHER_SWING.ENCODER_DIO_PIN);
  private ProfiledPIDController swingPID = new ProfiledPIDController(
    Constants.LAUNCHER_SWING.P_GAIN,
    Constants.LAUNCHER_SWING.I_GAIN,
    Constants.LAUNCHER_SWING.D_GAIN,
    null
  );
  private ArmFeedforward feedforward = new ArmFeedforward(
    Constants.LAUNCHER_SWING.FEEDFORWARD_S_GAIN, 
    Constants.LAUNCHER_SWING.FEEDFORWARD_G_GAIN, 
    Constants.LAUNCHER_SWING.FEEDFORWARD_V_GAIN
  );

  private Measure<Angle> selectedAngle;
  private Measure<Angle> angle;

  private XboxController m_driverController = new XboxController(Constants.ASO_CONTROLLER_PORT);

  public LauncherSwingSubsystem() 
  {

    this.swingMotor.setIdleMode(IdleMode.kBrake);

    swingPID.setIntegratorRange(-1, 1);

    swingEncoder.setDistancePerRotation(2*Math.PI);
    swingEncoder.setPositionOffset(0);

  }
  
  public Measure<Angle> getPosition(boolean selectedAngle){
    if(selectedAngle){
      return this.selectedAngle;
    } else{
    return Radians.of(swingEncoder.getDistance());
    }
  }

  public void setAngle(Measure<Angle> pos){
    selectedAngle = pos;
  }

  public Command setAngleCommand(Measure<Angle> pos){
    return runOnce(() -> {
      selectedAngle = pos;
    });
  }

  public boolean isAtSelectedPos(){
    if(Math.abs(selectedAngle.minus(angle).in(Degree)) <= Constants.LAUNCHER_SWING.POS_AE.in(Degree)){
      return true;
    } else {
      return false;
    }
  }

  public Command manualSwingControlCommand(){
    return run(() -> {
      selectedAngle = selectedAngle.plus(Degree.of((m_driverController.getLeftTriggerAxis()-m_driverController.getRightTriggerAxis())*Constants.LAUNCHER_SWING.MAN_CTRL_SENS));
    });
  }

  public void disabledPeriodic(){
    selectedAngle = angle;
  }
  @Override
  public void periodic() {
    updateTelemetry();

    angle = Radian.of(swingEncoder.getDistance());

    selectedAngle = clamp(selectedAngle, Constants.LAUNCHER_SWING.LOWER_BOUND, Constants.LAUNCHER_SWING.UPPER_BOUND);

    PIDOut = Volt.of(swingPID.calculate(angle.in(Radian), selectedAngle.in(Radian)));
    feedforwardOut = Volt.of(feedforward.calculate(swingPID.getSetpoint().position, swingPID.getSetpoint().velocity));
    
    swingMotor.setVoltage(clamp(
      PIDOut.plus(feedforwardOut).in(Volt),
     -(Constants.LAUNCHER_SWING.MAX_OUTPUT.in(Volt)), 
     Constants.LAUNCHER_SWING.MAX_OUTPUT.in(Volt)));
  
  }
  private void updateTelemetry(){
    SmartDashboard.putNumber ("launcher swing selected pos", selectedAngle.in(Degree));
    SmartDashboard.putNumber ("launcher swing current pos" , angle.in(Degree));
    SmartDashboard.putNumber ("PID voltage", PIDOut.in(Volt));
    
  }
  private double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }

  private Measure<Angle> clamp(Measure<Angle> val, Measure<Angle> min, Measure<Angle> max) {
    if(val.magnitude() > min.magnitude()){
      if(val.magnitude() < max.magnitude()){
        return val;
      } else {
        return max;
      }
    } else {
      return min;
    }
  }

}