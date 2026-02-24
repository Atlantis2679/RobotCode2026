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
    public final DoubleSupplier driveSpeedRPS = fields.addDouble("driveSpeedRPS",
            this::getDriveSpeedRPS);
    
    
    public final DoubleSupplier turnKP = fields.addDouble("turnKP", this::getTurnKP);
    public final DoubleSupplier turnKI = fields.addDouble("turnKI", this::getTurnKI);
    public final DoubleSupplier turnKD = fields.addDouble("turnKD", this::getTurnKD);

    public final DoubleSupplier current = fields.addDouble("Current", this::getCurrent);

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getDriveDistanceRotations();
    protected abstract double getDriveSpeedRPS();

    protected abstract double getAbsoluteTurnAngleRotations();
    protected abstract double getIntegratedTurnAngleRotations();

    protected abstract double getTurnKP();
    protected abstract double getTurnKI();
    protected abstract double getTurnKD();

    protected abstract double getCurrent();

    public abstract void setDriveVoltage(double voltage);
    public abstract void setDrivePercentageSpeed(double speed);

    public abstract void setTurnAngleRotations(double voltage);

    public abstract void setCoast();

    public abstract void setTurnKP(double kP);
    public abstract void setTurnKI(double kI);
    public abstract void setTurnKD(double kD);

    public abstract void resetIntegratedAngleRotations(double newAngle);
}
