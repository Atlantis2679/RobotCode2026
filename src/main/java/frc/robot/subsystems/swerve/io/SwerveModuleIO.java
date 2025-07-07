package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class SwerveModuleIO extends IOBase {
    public final DoubleSupplier angleRotations = fields.addDouble("angleRotations", this::getAngleRotations);

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getAngleRotations();

    public abstract void setDriveMotorVoltage(int voltage);

    public abstract void setTurnMotorVoltage(int voltage);
}
