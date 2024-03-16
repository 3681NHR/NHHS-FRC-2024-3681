// https://github.com/ligerbots/InfiniteRecharge2020/blob/infiniteSimulator/src/main/java/frc/robot/simulation/SparkMaxWrapper.java
/*
 * Create a wrapper around the CANSparkMax class to support simulation
 */

package frc.robot.simulation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;

public class SparkMaxWrapper extends CANSparkMax {
    private SimDouble m_simSpeed;
    private SimDevice m_simSparkMax;
    
    private RelativeEncoderWrapper m_simEncoder; 
    private SimDouble m_simPosition; 

    public SparkMaxWrapper(int deviceID, MotorType type) {
        super(deviceID,type);

        m_simSparkMax = SimDevice.create("SparkMax",deviceID);
        if (m_simSparkMax != null){
            m_simSpeed = m_simSparkMax.createDouble("speed", SimDevice.Direction.kBidir , 0.0);
        }

    }

    @Override
    public double get(){
        if (m_simSparkMax != null){
            return m_simSpeed.get();
        }
        return super.get();
    }

    @Override
    public void set(double speed){
        if (m_simSparkMax != null){
            m_simSpeed.set(speed);
            if (m_simEncoder != null && speed > 0.0) {
                m_simEncoder.setPosition(m_simEncoder.getPosition() + 0.01);
            }
        }else{
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (m_simSparkMax != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }

    @Override
    public RelativeEncoder getEncoder() {
        if (m_simSparkMax != null) {
            if (m_simEncoder == null) {
                m_simEncoder = new RelativeEncoderWrapper(this.getDeviceId()); 
            }
            return m_simEncoder;
        } else {
            return super.getEncoder();
        }
    }
}