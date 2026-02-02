package frc.robot.subsystems.intake.roller.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class RollerIOSim extends RollerIO {

    public RollerIOSim(LogFieldsTable fields) {
        super(fields);
    }

    public void setSpeed(double speed) {
    }

    protected double getCurrent() {
        return 0;
    }
}
