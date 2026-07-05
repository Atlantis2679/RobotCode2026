package frc.robot.subsystems.roller.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class RollerIO extends IOBase {
    public DoubleSupplier current = fields.addDouble("current", this::getCurrent);
    public DoubleSupplier position = fields.addDouble("position", this::getPosition);

    public RollerIO(LogFieldsTable fields) {
        super(fields);
    }

    protected abstract double getCurrent();

    protected abstract double getPosition();

    public abstract void setVoltage(double voltage);
}
