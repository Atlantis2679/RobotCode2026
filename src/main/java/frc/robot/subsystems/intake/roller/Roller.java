package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.roller.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Roller extends SubsystemBase {
    
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private RollerIO io = Robot.isReal() ? new RollerIOSparkMax(fieldsTable) : new RollerIOSim(fieldsTable);

    public Roller() {}


    @Override
    public void periodic(){
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public void stop(){
        io.setSpeed(0);
    }

    public void setSpeed(double speed){
        io.setSpeed(speed);
    }

    public double getCurrent(){
        return io.getCurrent.getAsDouble();
    }

}