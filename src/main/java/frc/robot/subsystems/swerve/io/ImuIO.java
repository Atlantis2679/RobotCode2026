package frc.robot.subsystems.swerve.io;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class ImuIO extends IOBase {
  public final DoubleSupplier angleDegreesCCW = fields.addDouble("angleDegreesCCW", this::getYawDegreesCCW);
  public final BooleanSupplier isConnected = fields.addBoolean("isGyroConnected", this::getIsConnected);
  
  public final DoubleSupplier xAcceleration = fields.addDouble("X Acceleration", this::getxAcceleration);
  public final DoubleSupplier yAcceleration = fields.addDouble("Y Acceleration", this::getyAcceleration);
  public final DoubleSupplier zAcceleration = fields.addDouble("Z Acceleration", this::getzAcceleration);


  public ImuIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  protected abstract double getYawDegreesCCW();

  protected abstract boolean getIsConnected();

  protected abstract double getxAcceleration();
  protected abstract double getyAcceleration();
  protected abstract double getzAcceleration();
}
