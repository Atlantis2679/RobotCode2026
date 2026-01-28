package frc.robot.subsystems.climber.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ClimberIO extends IOBase {
    public final DoubleSupplier elevatorMotorCurrect = fields.addDouble("elevatorMotorCurrect",
            this::getElevatorMotorCurrent);
    public final DoubleSupplier encoderAngle = fields.addDouble("encoderAngle", this::getEncoderAngleDegrees);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);


    public ClimberIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    
    protected abstract double getEncoderAngleDegrees();
    
    protected abstract double getElevatorMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    // Inputs:

    public abstract void setElevatorVoltage(double voltage);
}