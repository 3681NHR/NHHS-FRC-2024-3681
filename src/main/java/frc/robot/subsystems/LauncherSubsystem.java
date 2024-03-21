// done
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.LauncherState;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax m_left  = new CANSparkMax(Constants.LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_right = new CANSparkMax(Constants.LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);

  public LauncherState state = LauncherState.IDLE;

  public LauncherSubsystem() {
    
   
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
  @Override
  public void periodic() {

     this.m_left  .setIdleMode(IdleMode.kCoast);
    this.m_right  .setIdleMode(IdleMode.kCoast);

    SmartDashboard.putString("launcher state", state.toString());
    SmartDashboard.putNumber("L launcher speed", m_left.getEncoder().getVelocity());
    SmartDashboard.putNumber("R launcher speed", m_right.getEncoder().getVelocity());

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
  }

}
