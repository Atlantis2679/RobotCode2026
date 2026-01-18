package frc.robot.subsystems.shooter.flyWheel.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class FlyWheelIOSim extends FlyWheelIO{

    public FlyWheelIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    @Override
    public double getAbsoluteRotations() {
        return 0;
    }

    @Override
    public void setVoltage(double volt) {
    }
    
}
