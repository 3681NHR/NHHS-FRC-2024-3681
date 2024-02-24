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

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  public LauncherState state = LauncherState.IDLE;

  public LauncherSubsystem() {
   this.m_left  = new CANSparkMax(Constants.LAUNCHER_LEFT_MOTOR_ID, MotorType.kBrushless);
   this.m_right = new CANSparkMax(Constants.LAUNCHER_RIGHT_MOTOR_ID, MotorType.kBrushless);


    this.m_left  .setIdleMode(IdleMode.kBrake);
    this.m_right .setIdleMode(IdleMode.kBrake);
  }

  public Command toggleLaunch(){
    return runOnce(() -> {
      if(state == LauncherState.LAUNCHING){
        state = LauncherState.IDLE;
      } else{
        state = LauncherState.LAUNCHING;
      }
    });
  }
  public Command toggleDrop(){
    return runOnce(() -> {
      if(state == LauncherState.DROPPING){
        state = LauncherState.IDLE;
      } else{
        state = LauncherState.DROPPING;
      }
    });
  }
  
  @Override
  public void periodic() {

    SmartDashboard.putString("launcher state", state.toString());

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
    }
  }

}
