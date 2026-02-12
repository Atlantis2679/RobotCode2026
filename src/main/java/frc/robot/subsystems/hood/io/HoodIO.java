package frc.robot.subsystems.hood.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class HoodIO extends IOBase {
    public final DoubleSupplier motorRotations = fields.addDouble("motorRotations",
            this::getMotorRotations);
    public final BooleanSupplier limitSwitch = fields.addBoolean("limitSwitch", this::limitSwitch);
    public final DoubleSupplier motorCurrent = fields.addDouble("motorCurrent", motorRotations);

    public HoodIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getMotorRotations();

    protected abstract double getMotorCurrent();

    protected abstract boolean limitSwitch();

    public abstract void setCoast();

    public abstract void resetRotation(double rotations);

    public abstract void setVoltage(double volt);
}
