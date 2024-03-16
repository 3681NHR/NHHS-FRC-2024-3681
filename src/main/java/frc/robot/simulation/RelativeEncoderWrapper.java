package frc.robot.simulation;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class RelativeEncoderWrapper implements RelativeEncoder {

    private SimDouble m_simPosition;
    private SimDouble m_simVelocity;
    private SimDevice m_simRelativeEncoder;
    private SimDouble m_simPositionConversionFactor;

    RelativeEncoderWrapper(int deviceID) {
        m_simRelativeEncoder = SimDevice.create("RelativeEncoder", deviceID);
        if (m_simRelativeEncoder != null){
            m_simPosition = m_simRelativeEncoder.createDouble("Position", SimDevice.Direction.kBidir , 0.0);
            m_simVelocity = m_simRelativeEncoder.createDouble("Velocity", SimDevice.Direction.kBidir , 0.0);
            m_simPositionConversionFactor = m_simRelativeEncoder.createDouble("Position Conversion Factor", SimDevice.Direction.kBidir , 1.0);
        }
    }

    @Override
    public double getPosition() {
        if (m_simRelativeEncoder != null){
            // m_simPosition.set(m_simPosition.get() + 0.01);
            return m_simPosition.get();
        }
        return 0.0;
    }

    @Override
    public double getVelocity() {
        if (m_simRelativeEncoder != null){
            return m_simVelocity.get();
        }
        return 0.0;
    }

    @Override
    public REVLibError setPosition(double position) {
        if (m_simRelativeEncoder != null){
            m_simPosition.set(position);
            return REVLibError.kOk;
        }
        return REVLibError.kError;
        // else{
        //     super.set(position);
        // }
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPositionConversionFactor'");
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocityConversionFactor'");
    }

    @Override
    public double getPositionConversionFactor() {
        if (m_simRelativeEncoder != null){
            return m_simPositionConversionFactor.get();
        }
        return 0.0;
    }

    @Override
    public double getVelocityConversionFactor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityConversionFactor'");
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAverageDepth'");
    }

    @Override
    public int getAverageDepth() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAverageDepth'");
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMeasurementPeriod'");
    }

    @Override
    public int getMeasurementPeriod() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurementPeriod'");
    }

    @Override
    public int getCountsPerRevolution() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCountsPerRevolution'");
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getInverted'");
    }
    
}
