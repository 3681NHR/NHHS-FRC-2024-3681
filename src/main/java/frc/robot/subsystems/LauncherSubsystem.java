// done
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.LauncherState;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_left  = new CANSparkMax(Constants.LAUNCHER.LEFT_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_right = new CANSparkMax(Constants.LAUNCHER.RIGHT_MOTOR_ID, MotorType.kBrushless);

  public LauncherState state = LauncherState.IDLE;
  public LauncherSubsystem() {
  }

  public LauncherState getState(){
    return state;
  }
  
  public double getSpeed(boolean right){
    if(right){
      return m_right.getEncoder().getVelocity();
    } else {
      return m_left.getEncoder().getVelocity();
    }
  }

  public void setState(LauncherState s){
    state = s;
  }
  @Override
  public void periodic() {
    
    this.m_left  .setIdleMode(IdleMode.kCoast);
    this.m_right  .setIdleMode(IdleMode.kCoast);
    SmartDashboard.putString("launcher state", state.toString());
    SmartDashboard.putNumber("L launcher speed", m_left.getEncoder().getVelocity());
    SmartDashboard.putNumber("R launcher speed", m_right.getEncoder().getVelocity());

    switch(state){
      case LAUNCHING:
        m_left  .set(Constants.LAUNCHER.LAUNCH_OUTPUT);
        m_right .set(-Constants.LAUNCHER.LAUNCH_OUTPUT);
        break;
      case DROPPING:
        m_left  .set(Constants.LAUNCHER.DROP_OUTPUT);
        m_right .set(-Constants.LAUNCHER.DROP_OUTPUT);
        break;
      case IDLE:
        m_left  .set(0);
        m_right .set(0);
        break;
      case IN:
        m_left .set(Constants.LAUNCHER.IN_OUTPUT);
        m_right.set(-Constants.LAUNCHER.IN_OUTPUT);
    }
  }
}
