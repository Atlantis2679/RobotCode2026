package frc.robot.subsystems.intake.slapdown.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class SlapdownIOSim extends SlapdownIO {
    
    public SlapdownIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    protected double getAngleDegrees(){
        return 0;
    }
    protected boolean isEncoderConnected(){
        return false;
    }
    protected double getCurrent(){
        return 0;
    }

    public void setVolt(double volt){}
}
