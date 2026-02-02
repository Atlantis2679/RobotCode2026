package frc.robot.subsystems.intake.roller.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class RollerIO extends IOBase {
    public DoubleSupplier current = fields.addDouble("current", this::getCurrent);

    public RollerIO(LogFieldsTable fields) {
        super(fields);
    }

    protected abstract double getCurrent();

    public abstract void setSpeed(double speed);
}
