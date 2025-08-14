package frc.robot.subsystems.swerve.io;


import com.studica.frc.AHRS;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class GyroIONavX extends GyroIO {
  private final AHRS navX = new AHRS(new NavxCom());

  public GyroIONavX(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  @Override
  protected double getAngleDegreesCW() {

  }
}