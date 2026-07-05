package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.CONSTANT_VOLTAGE;
import static frc.robot.subsystems.roller.RollerConstants.KD;
import static frc.robot.subsystems.roller.RollerConstants.KI;
import static frc.robot.subsystems.roller.RollerConstants.KP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.roller.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;

public class Roller extends SubsystemBase implements Tunable {
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private RollerIO io = Robot.isReal() ? new RollerIOTalon(fieldsTable) : new RollerIOSim(fieldsTable);
    private final PIDController anglePID = new PIDController(KP, KI, KD);
    private double constantVoltage = CONSTANT_VOLTAGE;

    public Roller() {
        TunablesManager.add("Roller", (Tunable) this);
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("position", getPosition());
        fieldsTable.recordOutput("current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public void stop() {
        io.setVoltage(0);
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -RollerConstants.MAX_VOLTAGE, RollerConstants.MAX_VOLTAGE);
        io.setVoltage(voltage);
    }

    public double getPosition() {
        return io.position.getAsDouble();
    }

    public double calculatePositionVoltage(double angle) {
        return anglePID.calculate(getPosition(), angle);
    }

    public double getCurrent() {
        return io.current.getAsDouble();
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("PID Controller", anglePID);
        builder.addDoubleProperty("Constant Voltage", () -> constantVoltage, (value) -> constantVoltage = value);
    }
}