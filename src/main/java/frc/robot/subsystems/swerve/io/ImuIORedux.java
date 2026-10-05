package frc.robot.subsystems.swerve.io;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.RobotMap.CANBUS.CAN_AND_GYRO_ID;

public class ImuIORedux extends ImuIO {
    private final Canandgyro canandgyro = new Canandgyro(CAN_AND_GYRO_ID);

    public ImuIORedux(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected double getYawDegreesCCW() {
        return canandgyro.getYaw() * 360 + 180;
    }

    @Override
    protected double getPitchDeg() {
        return canandgyro.getPitch() * 360 + 180;
    }

    @Override
    protected double getRollDeg() {
        return canandgyro.getRoll() * 360 + 180;
    }

    @Override
    protected boolean getIsConnected() {
        return canandgyro.isConnected();
    }

    @Override
    protected double getXAcceleration() {
        return canandgyro.getAccelerationX();
    }

    @Override
    protected double getYAcceleration() {
        return canandgyro.getAccelerationY();
    }

    @Override
    protected double getZAcceleration() {
        return canandgyro.getAccelerationZ();
    }   
}
