package frc.robot.subsystems.climber.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ClimberIOSim extends ClimberIO {
    public ClimberIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    public double getEncoderAngleDegrees(){
        return 0;
    }

    public double getHeightMeters() {
        return 0;
    }
    public double getElevatorMotorCurrent() {
        return 0;
    }
    public double getPivotMotorCurrent() {
        return 0;
    }
    protected boolean getIsEncoderConnected() {
        return true;
    }
    public void setElevatorVoltage(double voltage) {}
    public void setPivotVoltage(double voltage) {}
}
