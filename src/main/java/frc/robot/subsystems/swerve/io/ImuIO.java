package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ImuIO extends IOBase {
  public final DoubleSupplier angleDegreesCCW = fields.addDouble("angleDegreesCCW", this::getYawDegreesCCW);
  public final BooleanSupplier isConnected = fields.addBoolean("isGyroConnected", this::getIsConnected);

  public ImuIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getYawDegreesCCW();

  protected abstract boolean getIsConnected();
}
