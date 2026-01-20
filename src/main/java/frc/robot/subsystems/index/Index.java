package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.index.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Index extends SubsystemBase{
    private final LogFieldsTable fields = new LogFieldsTable(getName());
    private final IndexIO io = Robot.isReal() ? new IndexIOSparkMax(fields) : new IndexIOSim(fields);
    
    public Index(){}

    @Override
    public void periodic(){
        fields.recordOutput("Current command", 
            getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public void stop(){
        io.setSpindexVolt(0);
        io.setIndexerVolt(0);
    }

    //Input:
    public void setSpindexVolt(double volt){
        fields.recordOutput("Spin Motor volt", volt);
        io.setSpindexVolt(volt);
    }
    public void setIndexerVolt(double volt){
        fields.recordOutput("In Motor Volt", volt);
        io.setIndexerVolt(volt);
    }

    //Output:
    public double getSpindexCurrent() {
        return io.getSpindexCurrent.getAsDouble();
    }
    public double getIndexerCurrent() {
        return io.getIndexerCurrent.getAsDouble();
    }
}
