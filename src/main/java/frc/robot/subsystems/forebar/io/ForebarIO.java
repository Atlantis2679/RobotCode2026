package frc.robot.subsystems.forebar.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ForebarIO extends IOBase {
    public DoubleSupplier current = fields.addDouble("Current", this::getCurrent);
    public BooleanSupplier isEncoderConnected = fields.addBoolean("Encoder Connected", this::isEncoderConnected);
    public DoubleSupplier angleDegrees = fields.addDouble("Angle Degrees", this::getAngleDegrees);

    public ForebarIO(LogFieldsTable fields) {
        super(fields);
    }

    protected abstract double getCurrent();

    protected abstract boolean isEncoderConnected();

    protected abstract double getAngleDegrees();

    public abstract void setVolt(double volt);
}
