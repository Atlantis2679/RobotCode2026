package frc.robot.subsystems.intake.slapdown.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class SlapdownIO extends IOBase{
    private LogFieldsTable fields;

    public DoubleSupplier getCurrent = fields.addDouble("Current", this::getCurrent);
    public BooleanSupplier isEncoderConnected = fields.addBoolean("Encoder Connected?", this::isEncoderConnected);
    public DoubleSupplier getAngle = fields.addDouble("Angle", this::getAngle);

    public SlapdownIO(LogFieldsTable fields){
        super(fields);
    }

    protected abstract double getCurrent();
    protected abstract boolean isEncoderConnected();
    protected abstract double getAngle();

    public abstract void setVolt(double volt);
}
