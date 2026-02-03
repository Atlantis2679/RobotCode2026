package frc.robot.subsystems.forebar.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ForebarIOSim extends ForebarIO {
    public ForebarIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected double getAngleDegrees() {
        return 0;
    }

    protected boolean isEncoderConnected() {
        return false;
    }

    protected double getCurrent() {
        return 0;
    }

    public void setVolt(double volt) {
    }
}
