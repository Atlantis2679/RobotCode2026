package frc.robot.subsystems.roller.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class RollerIOSim extends RollerIO {

    public RollerIOSim(LogFieldsTable fields) {
        super(fields);
    }

    public void setVoltage(double voltage) {
    }

    protected double getCurrent() {
        return 0;
    }
}
