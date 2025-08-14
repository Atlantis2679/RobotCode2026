package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class GyroIO extends IOBase {
  public final DoubleSupplier angleDegreesCw = fields.addDouble("angleDegreesCW", this::getAngleDegreesCW);

  public GyroIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getAngleDegreesCW();
}
