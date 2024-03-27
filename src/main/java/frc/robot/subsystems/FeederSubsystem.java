// done
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  private VictorSPX roller = new VictorSPX(Constants.INDEXER.MOTOR_ID);
  private DigitalInput holdingSwitch = new DigitalInput(Constants.INDEXER.DETECTOR_DIO_PIN);

  private boolean holding = false;
  private boolean switchEnabled = true;

  public FeederSubsystem() {this.roller.setNeutralMode(NeutralMode.Brake);}

  public void setspeed(double val){roller.set(ControlMode.PercentOutput, val);}

  public double getspeed(){return roller.getMotorOutputPercent();}

  public boolean isHolding(){
    if(switchEnabled){
      return holding;
    } else {
      return true;
    }
  }

  public boolean getSwitchEnabled(){return switchEnabled;}

  public void setSwitchEnabled(boolean enabled){switchEnabled = enabled;}

  @Override
  public void periodic() {
    holding = !holdingSwitch.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("feeder holding", this::isHolding, null);
    builder.addBooleanProperty("feeder switch enabled", this::getSwitchEnabled, this::setSwitchEnabled);
    builder.addDoubleProperty("feeder output percent", roller::getMotorOutputPercent, null);
    builder.addDoubleProperty("feeder output voltage", roller::getMotorOutputVoltage, null);
  }
}
