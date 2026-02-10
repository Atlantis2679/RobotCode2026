package frc.robot.subsystems.swerve.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ImuIOSim extends ImuIO {
    public ImuIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected double getYawDegreesCCW() {
        return 0;
    }

    @Override
    protected boolean getIsConnected() {
        return false;
    }

    @Override
    protected double getXAcceleration() {
    return 0;
    }
    
    @Override
    protected double getYAcceleration() {
    return 0;
    }
    
    @Override
    protected double getZAcceleration() {
    return 0;
    }

    @Override
    protected double getAcceleration() {
        return 0;
    }

}
