// done
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  private VictorSPX roller = new VictorSPX(Constants.INDEXER.MOTOR_ID);
  private DigitalInput holdingSwitch = new DigitalInput(Constants.INDEXER.DETECTOR_DIO_PIN);

  private boolean holding = false;
  private boolean switchEnabled = true;

  public IndexerSubsystem() {
    SmartDashboard.putBoolean("indexer switch Enabled", switchEnabled);

    this.roller.setNeutralMode(NeutralMode.Brake);
  }

  public void setspeed(double val){
    roller.set(ControlMode.PercentOutput, val);
  }

  public double getspeed(){
    return roller.getMotorOutputPercent();
  }

  public boolean isHolding(){
    if(switchEnabled){
      return holding;
    } else {
      return true;
    }
  }

  public boolean getSwitchEnabled(){
    return switchEnabled;
  }

  public void setSwitchEnabled(boolean enabled){
    switchEnabled = enabled;
  }

  @Override
  public void periodic() {
    
    holding = !holdingSwitch.get();

    SmartDashboard.putBoolean("indexer Holding", holding);

    switchEnabled = SmartDashboard.getBoolean("launcher switch Enabled", false);

  }
}
