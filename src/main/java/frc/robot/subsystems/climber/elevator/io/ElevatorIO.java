package frc.robot.subsystems.climber.elevator.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase {
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent",
            this::getMotorCurrent);
    public final DoubleSupplier heightMeters = fields.addDouble("height", this::getHeightMeters);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);


    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getHeightMeters();
    
    protected abstract double getMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    public abstract void setVoltage(double voltage);
}