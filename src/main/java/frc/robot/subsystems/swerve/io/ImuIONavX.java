package frc.robot.subsystems.swerve.io;

import com.studica.frc.AHRS;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ImuIONavX extends ImuIO {
  private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private static final boolean GYRO_UPSIDE_DOWN = true;

  public ImuIONavX(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  @Override
  protected double getYawDegreesCCW() {
    return (GYRO_UPSIDE_DOWN ? 1 : -1) * navX.getAngle();
  }

  @Override
  protected boolean getIsConnected() {
    return navX.isConnected();
  }
  
  @Override
  protected double getXAcceleration() {
    return navX.getWorldLinearAccelX();
  }

  @Override
  protected double getYAcceleration() {
    return navX.getWorldLinearAccelY();
  }

  @Override
  protected double getZAcceleration() {
    return navX.getWorldLinearAccelZ();
  }

  @Override
  protected double getPitchDeg() {
    return navX.getPitch();
  }

  @Override
  protected double getRollDeg() {
    return navX.getRoll();
  }

}