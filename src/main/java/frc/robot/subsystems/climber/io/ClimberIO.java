package frc.robot.subsystems.climber.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ClimberIO extends IOBase {
    public final DoubleSupplier rightMotorCurrent = fields.addDouble("elevatorMotorCurrect",
            this::getElevatorMotorCurrent);
    public final DoubleSupplier leftMotorCurrent = fields.addDouble("pivotMotorCurrect",
            this::getPivotMotorCurrent);
    public final DoubleSupplier height = fields.addDouble("elevatorHeight", this::getHeightMeters);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);


    public ClimberIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    
    public abstract double getEncoderAngleDegrees();

    public abstract double getHeightMeters();
    
    public abstract double getElevatorMotorCurrent();

    public abstract double getPivotMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    // Inputs:

    public abstract void setElevatorVoltage(double voltage);

    public abstract void setPivotVoltage(double voltage);

}