package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ImuIO extends IOBase {
  public final DoubleSupplier angleDegreesCCW = fields.addDouble("angleDegreesCCW", this::getYawDegreesCCW);
  public final BooleanSupplier isConnected = fields.addBoolean("isGyroConnected", this::getIsConnected);

  public final DoubleSupplier xAcceleration = fields.addDouble("X Acceleration", this::getXAcceleration);
  public final DoubleSupplier yAcceleration = fields.addDouble("Y Acceleration", this::getYAcceleration);
  public final DoubleSupplier zAcceleration = fields.addDouble("Z Acceleration", this::getZAcceleration);

  public ImuIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getYawDegreesCCW();

  protected abstract boolean getIsConnected();

  protected abstract double getXAcceleration();
  protected abstract double getYAcceleration();
  protected abstract double getZAcceleration();
}
