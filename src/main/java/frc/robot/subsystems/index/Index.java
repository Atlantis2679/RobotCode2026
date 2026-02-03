package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.index.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Index extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final IndexIO io = Robot.isReal() ? new IndexIOSparkMax(fieldsTable) : new IndexIOSim(fieldsTable);

    public Index() {
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public void stop() {
        io.setSpindexVolt(0);
        io.setIndexerVolt(0);
    }

    public void setSpindexVolt(double volt) {
        fieldsTable.recordOutput("Spindex Motor Desired Volt", volt);
        io.setSpindexVolt(volt);
    }

    public void setIndexerVolt(double volt) {
        fieldsTable.recordOutput("Indexer Motor Desired Volt", volt);
        io.setIndexerVolt(volt);
    }

    public double getSpindexCurrent() {
        return io.indexerCurrent.getAsDouble();
    }

    public double getIndexerCurrent() {
        return io.indexerCurrent.getAsDouble();
    }
}
