package frc.robot.subsystems.climber.elevator.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase {
    public final DoubleSupplier elevatorMotorCurrect = fields.addDouble("elevatorMotorCurrect",
            this::getElevatorMotorCurrent);
    public final DoubleSupplier elevatorHeight = fields.addDouble("elevatorHeight", this::getElevatorHeight);
    public final BooleanSupplier isEncoderConnected = fields.addBoolean("isEncoderConnected", this::getIsEncoderConnected);


    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // Outputs:
    
    protected abstract double getElevatorHeight();
    
    protected abstract double getElevatorMotorCurrent();

    protected abstract boolean getIsEncoderConnected();

    // Inputs:

    public abstract void setElevatorVoltage(double voltage);
}