package frc.robot.subsystems.fourbar;

import static frc.robot.subsystems.fourbar.FourbarConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.fourbar.io.FourbarIO;
import frc.robot.subsystems.fourbar.io.FourbarIOSim;
import frc.robot.subsystems.fourbar.io.FourbarIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;

public class Fourbar extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final FourbarIO io = Robot.isReal() ? new FourbarIOSparkMax(fieldsTable) : new FourbarIOSim(fieldsTable);
    private final RotationalSensorHelper angleDegrees = new RotationalSensorHelper(io.angleDegrees.getAsDouble());

    private final Debouncer isStuckDebouncer = new Debouncer(STUCK_DEBOUNCE_SEC, DebounceType.kRising);

    private double desiredVoltage = 0;

    public Fourbar() {
        TunablesManager.add(getName(), (Tunable) this);
    }


    @Override
    public void periodic() {
        angleDegrees.update(io.angleDegrees.getAsDouble());
        fieldsTable.recordOutput("Desired Voltage", desiredVoltage);
        fieldsTable.recordOutput("isStuck", isStuck());
        fieldsTable.recordOutput("angle", getAngleDegrees());
        fieldsTable.recordOutput("velocity", getVelocity());
        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public double getCurrent() {
        return io.current.getAsDouble();
    }

    public double getAngleDegrees() {
        return angleDegrees.getAngle();
    }

    public double getVelocity() {
        return angleDegrees.getVelocity();
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        desiredVoltage = voltage;
        io.setVolt(voltage);
    }

    public void stop() {
        desiredVoltage = 0;
        io.setVolt(0);
    }

    public boolean isStuck() {
        return isStuckDebouncer.calculate(
                Math.abs(desiredVoltage) > 0 && Math.abs(getVelocity()) < STUCK_VELOCITY_THRESHOLD_DEG_PER_SEC);
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Forbar RotationalSensorHelper", angleDegrees);
    }
}
