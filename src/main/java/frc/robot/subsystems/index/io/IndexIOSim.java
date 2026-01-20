package frc.robot.subsystems.index.io;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class IndexIOSim extends IndexIO{

    public IndexIOSim(LogFieldsTable fields){
        super(fields);
    }

    //Input:
    public void setSpindexVolt(double volt){}
    public void setIndexerVolt(double volt){}

    //Output:
    protected double getSpindexCurrent(){return 0;}
    protected double getIndexerCurrent(){return 0;}
}
