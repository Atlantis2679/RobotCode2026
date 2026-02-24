package frc.robot.subsystems.hood.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class HoodIO extends IOBase {
    public final DoubleSupplier motorRotations = fields.addDouble("motorRotations",
            this::getMotorRotations);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", this::getMotorCurrent);
    public final DoubleSupplier absoluteAngleDegrees = fields.addDouble("absoluteAngleDegrees", this::getAbsolueAngleDegrees);

    public HoodIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getMotorRotations();

    protected abstract double getAbsolueAngleDegrees();

    protected abstract double getMotorCurrent();

    public abstract void setCoast();

    public abstract void setVoltage(double volt);

    public abstract void setCurrentLimit(double currentLimit);
}
