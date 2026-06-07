package frc.robot.subsystems.index.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class IndexIOSim extends IndexIO{

    public IndexIOSim(LogFieldsTable fields){
        super(fields);
    }

    public void setSpindexVolt(double volt) {}
    public void setIndexerVolt(double volt) {}

    protected double getSpindexFollowerCurrent() {
        return 0;
    }

    protected double getSpindexLeaderCurrent() {
        return 0;
    }


    protected double getIndexerCurrent() {
        return 0;
    }
}
