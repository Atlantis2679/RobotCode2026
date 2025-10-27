package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class SwerveModuleIO extends IOBase {
    public final DoubleSupplier absoluteTurnAngleRotations = fields.addDouble("absoluteTurnAngleRotations",
            this::getAbsoluteTurnAngleRotations);
        public final DoubleSupplier intergatedTurnAngleRotations = fields.addDouble("intergatedTurnAngleRotations",
            this::getIntegratedTurnAngleRotations);
    public final DoubleSupplier driveDistanceRotations = fields.addDouble("driveDistanceRotations",
            this::getDriveDistanceRotations);

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getAbsoluteTurnAngleRotations();

    protected abstract double getDriveDistanceRotations();

    protected abstract double getIntegratedTurnAngleRotations();

    public abstract void setDriveVoltage(double voltage);

    public abstract void setDrivePercentageSpeed(double speed);

    public abstract void setTurnAngleRotations(double voltage);

    public abstract void setCoast();
}
