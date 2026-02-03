package frc.robot.subsystems.hood.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class HoodIO extends IOBase {
    public final DoubleSupplier motorAngleDegrees = fields.addDouble("motorAngleDegrees",
            this::getHoodMotorAngleDegree);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected",
            this::getIsEncoderConnected);

    public HoodIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getHoodMotorAngleDegree();

    protected abstract boolean getIsEncoderConnected();

    public abstract void setVoltage(double volt);
}
