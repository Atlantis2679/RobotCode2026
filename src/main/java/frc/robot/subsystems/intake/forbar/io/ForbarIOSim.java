package frc.robot.subsystems.intake.forbar.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ForbarIOSim extends ForbarIO {
    public ForbarIOSim(LogFieldsTable fieldsTable) {
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
