package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GyroIO extends IOBase {
  public final DoubleSupplier angleDegreesCw = fields.addDouble("angleDegreesCW", this::getYawDegreesCW);
  public final BooleanSupplier isConnected = fields.addBoolean("isGyroConnected", this::getIsConnected);

  public GyroIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getYawDegreesCW();

  protected abstract boolean getIsConnected();
}
