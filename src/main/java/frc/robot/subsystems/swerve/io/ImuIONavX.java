package frc.robot.subsystems.swerve.io;

import com.studica.frc.AHRS;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ImuIONavX extends ImuIO {
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

  public ImuIONavX(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  @Override
  protected double getYawDegreesCCW() {
    return -navX.getAngle();
  }

  @Override
  protected boolean getIsConnected() {
    return navX.isConnected();
  }

  @Override
  protected double getxAcceleration() {
    return navX.getRawAccelX();
  }

  @Override
  protected double getyAcceleration() {
    return navX.getRawAccelY();
  }
  @Override
  protected double getzAcceleration() {
    return navX.getRawAccelZ();
  }
}