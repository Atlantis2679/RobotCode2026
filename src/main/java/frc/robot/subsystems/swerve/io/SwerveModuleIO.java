package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class SwerveModuleIO extends IOBase {
    public final DoubleSupplier absoluteAngleRotations = fields.addDouble("absoluteAngleRotations", this::getAbsoluteAngleRotations);

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getAbsoluteAngleRotations();

    public abstract void setDriveVoltage(double voltage);

    public abstract void setTurnVoltage(double voltage);

    public abstract void setDrivePercentageSpeed(double speed);

    public abstract void setTurnPercentageSpeed(double speed);
}
