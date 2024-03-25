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

public class IndexerSubsystem extends SubsystemBase {

  private VictorSPX   roller=   new VictorSPX(Constants.LAUNCHER.ROLLER_MOTOR_ID);
  private DigitalInput holdingSwitch = new DigitalInput(Constants.LAUNCHER.DETECTOR_DIO_PIN);

  private boolean holding = false;
  private boolean switchEnabled = true;

  public IndexerSubsystem() {
    SmartDashboard.putBoolean("indexer switch Enabled", switchEnabled);
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
    if(switchEnabled){
      return holding;
    } else {
      return true;
    }
  }
  public boolean switchEnabled(){
    return switchEnabled;
  }

  @Override
  public void periodic() {
    
    holding = !holdingSwitch.get();

    this.roller    .setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putBoolean("indexer Holding", holding); 
    SmartDashboard.putString("indexer state", rollerState.toString());

    switchEnabled = SmartDashboard.getBoolean("launcher switch Enabled", false);

  }
}
