package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GyroIO extends IOBase {
  public final DoubleSupplier angleDegreesCCW = fields.addDouble("angleDegreesCW", this::getYawDegreesCCW);
  public final BooleanSupplier isConnected = fields.addBoolean("isGyroConnected", this::getIsConnected);

  public GyroIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getYawDegreesCCW();

  protected abstract boolean getIsConnected();
}
