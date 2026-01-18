package frc.robot.subsystems.shooter.flyWheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.flyWheel.io.*;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class FlyWheel extends SubsystemBase{
    
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final FlyWheelIO io = Robot.isReal()?new FlyWheelIOSparkMax(fieldsTable) :
        new FlyWheelIOSim(fieldsTable);

    private final RotationalSensorHelper sensorHelper;
    
    public FlyWheel() {
        fieldsTable.update();
        
        sensorHelper = new RotationalSensorHelper(io.flywheelMotorAbsoluteRotations.getAsDouble(), 0);
    }

    @Override
    public void periodic(){
        sensorHelper.update(io.flywheelMotorAbsoluteRotations.getAsDouble());

        fieldsTable.recordOutput("current command", getCurrentCommand() == null?
        getCurrentCommand().getName() : "None");

        SmartDashboard.putNumber("FlyWheel abs rotations", getAbsoluteRotations());
    }


    public double getAbsoluteRotations(){
        return io.getAbsoluteRotations();
    }
    public double getVelocity(){
        return sensorHelper.getVelocity();
    }
    public void setVoltage(double volt){
        io.setVoltage(volt);
    }
    public void stop(){
        io.setVoltage(0);
    }
    
}
