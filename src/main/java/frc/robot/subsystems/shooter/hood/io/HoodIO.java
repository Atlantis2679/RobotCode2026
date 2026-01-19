package frc.robot.subsystems.shooter.hood.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class HoodIO extends IOBase{
    
    public final DoubleSupplier hoodMotorAngle = fields.addDouble("Hood Motor Angle",
    this::getHoodMotorAngle);

    public HoodIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    public abstract double getHoodMotorAngle();

    public abstract void setVoltage(double volt);
}
