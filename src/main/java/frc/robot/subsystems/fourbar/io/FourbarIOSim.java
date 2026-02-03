package frc.robot.subsystems.fourbar.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class FourbarIOSim extends FourbarIO {
    public FourbarIOSim(LogFieldsTable fieldsTable) {
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
