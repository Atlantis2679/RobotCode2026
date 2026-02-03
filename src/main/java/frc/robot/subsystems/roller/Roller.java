package frc.robot.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.roller.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Roller extends SubsystemBase {
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private RollerIO io = Robot.isReal() ? new RollerIOSparkMax(fieldsTable) : new RollerIOSim(fieldsTable);

    public Roller() {
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public void stop() {
        io.setSpeed(0);
    }

    public void setSpeedRPM(double speed) {
        speed = MathUtil.clamp(speed, -RollerConstants.MAX_RPM, RollerConstants.MAX_RPM);
        io.setSpeed(speed / RollerConstants.MAX_RPM);
    }

    public double getCurrent() {
        return io.current.getAsDouble();
    }
}