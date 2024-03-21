// done
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.LauncherState;
import frc.robot.enums.RollerState;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_left  = new CANSparkMax(Constants.LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_right = new CANSparkMax(Constants.LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private VictorSPX   roller      = new VictorSPX(Constants.LAUNCHER_ROLLER_MOTOR_ID);

  private DigitalInput holdingSwitch = new DigitalInput(Constants.LAUNCHER_DETECTOR_DIO_PIN);
  private boolean holding = false;

  private boolean LaunchLock = true;

  public LauncherState state = LauncherState.IDLE;
  private RollerState rollerState = RollerState.IDLE;

  public LauncherSubsystem() {
  }

  public void setRoller(RollerState state){
    rollerState = state;
  }

  public Command setRollerCommand(RollerState state){
    return runOnce(() -> {
      rollerState = state;
    });
  }

  public Command runRollersCommand() {
    return runOnce(
    () -> {
    if(holding){
       rollerState = RollerState.LAUNCH;
     } else{
        rollerState = RollerState.RECV;
      } 
    });
  }  

  public boolean isHolding(){
    return holding;
  }

  public LauncherState getState(){
    return state;
  }
  public double getSpeed(){
    return m_left.getEncoder().getVelocity();
  }

  public Command setSpeedCommand(LauncherState s){
    return runOnce(() -> {
      state = s;
    });
  }
  public void setSpeed(LauncherState s){
    state = s;
  }
  public boolean atspeed(){
    if(state == LauncherState.LAUNCHING){
      return m_left.getEncoder().getVelocity() >= Constants.LAUNCHER_LAUNCH_SPEED_RPM;
    } else if(state == LauncherState.DROPPING){
      return m_left.getEncoder().getVelocity() >= Constants.LAUNCHER_DROP_SPEED_RPM;
    } else {
      return false;
    }
  }
  @Override
  public void periodic() {
    
    holding = !holdingSwitch.get();

    this.m_left  .setIdleMode(IdleMode.kCoast);
    this.m_right  .setIdleMode(IdleMode.kCoast);
    this.roller    .setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putString("launcher state", state.toString());
    SmartDashboard.putNumber("L launcher speed", m_left.getEncoder().getVelocity());
    SmartDashboard.putNumber("R launcher speed", m_right.getEncoder().getVelocity());
    SmartDashboard.putBoolean("LauncherIsHolding", holding); 

    switch(state){
      case LAUNCHING:
        m_left  .set(Constants.LAUNCHER_LAUNCH_SPEED);
        m_right .set(-Constants.LAUNCHER_LAUNCH_SPEED);
        break;
      case DROPPING:
        m_left  .set(Constants.LAUNCHER_DROP_SPEED);
        m_right .set(-Constants.LAUNCHER_DROP_SPEED);
        break;
      case IDLE:
        m_left  .set(0);
        m_right .set(0);
        break;
      case IN:
        m_left .set(Constants.LAUNCHER_IN_SPEED);
        m_right.set(-Constants.LAUNCHER_IN_SPEED);
    }
    switch(rollerState){
      case RECV:
        if(!holding){
          roller.set(ControlMode.PercentOutput, Constants.LAUNCHER_ROLLER_RECV_SPEED);
        } else {
          rollerState = RollerState.IDLE;
        }
        break;
      case BACKOUT:
        roller.set(ControlMode.PercentOutput, Constants.LAUNCHER_ROLLER_BACKOUT_SPEED);
        break;
      case IDLE:
        roller.set(ControlMode.PercentOutput, 0);
        break;
      case LAUNCH:
        if(!LaunchLock || atspeed()){
          roller.set(ControlMode.PercentOutput, Constants.LAUNCHER_ROLLER_RECV_SPEED);
        } else {
          rollerState = RollerState.IDLE;
        }
        break;
    }
  }

}
