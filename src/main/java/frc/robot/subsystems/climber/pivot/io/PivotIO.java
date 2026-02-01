package frc.robot.subsystems.climber.pivot.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class PivotIO extends IOBase{
    public final DoubleSupplier elevatorMotorCurrect = fields.addDouble("pivotMotorCurrent",
            this::getPivotMotorCurrent);
    public final DoubleSupplier pivotAngleDegrees = fields.addDouble("pivotAngleDegrees", this::getPivotAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);

    public PivotIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    protected abstract double getPivotAngleDegrees();
    
    protected abstract double getPivotMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    // Inputs:

    public abstract void setPivotVoltage(double voltage);
}

