package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.hood.io.HoodIO;
import frc.robot.subsystems.hood.io.HoodIOSim;
import frc.robot.subsystems.hood.io.HoodIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class Hood extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final HoodIO io = Robot.isReal() ? new HoodIOSparkMax(fieldsTable) : new HoodIOSim(fieldsTable);

    private final HoodVisualizer realVisualizer = new HoodVisualizer(fieldsTable, "Real Visualizer",
            new Color8Bit(Color.kPurple));
    private final HoodVisualizer desiredHoodVisualizer = new HoodVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));

    private final RotationalSensorHelper angleDegrees;

    private final PIDController pid = new PIDController(KP, KI, KD);

    private final Debouncer limitSwitchDebouncer = new Debouncer(LIMIT_SWITCH_DEBAUNCER_SEC);

    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private boolean calibrated = false;

    public Hood() {
        fieldsTable.update();

        TunablesManager.add("Hood", (Tunable) this);

        angleDegrees = new RotationalSensorHelper(io.motorRotations.getAsDouble() * GEAR_RATIO, ANGLE_OFFSET);
    }

    public void periodic() {
        angleDegrees.update(io.motorRotations.getAsDouble() * GEAR_RATIO);
        realVisualizer.update(getAngleDegrees());
        fieldsTable.recordOutput("angle", getAngleDegrees());
        fieldsTable.recordOutput("velocity", getVelocity());
        fieldsTable.recordOutput("limitSwitchValue", getLimitSwitchValue());
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        if (getLimitSwitchValue()) {
            resetAngleDegrees(0);
            calibrated = true;
        }
    }

    public double calculatePID(double desiredAngleDegrees) {
        if (desiredAngleDegrees < minAngle || desiredAngleDegrees > maxAngle) return 0.0;
        if (isAtAngle(desiredAngleDegrees)) return 0.0;
        fieldsTable.recordOutput("Desired angle PID", desiredAngleDegrees);
        desiredHoodVisualizer.update(desiredAngleDegrees);
        return pid.calculate(getAngleDegrees(), desiredAngleDegrees);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public void coast() {
        io.setCoast();
    }

    public double getAngleDegrees() {
        return angleDegrees.getAngle();
    }

    public double getVelocity() {
        return angleDegrees.getVelocity();
    }

    public void resetPID() {
        pid.reset();
    }

    public boolean isAtAngle(double desiredAngleDegrees) {
        return Math.abs(desiredAngleDegrees - getAngleDegrees()) < ANGLE_TOLERENCE_DEGREES;
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public boolean getLimitSwitchValue() {
        return limitSwitchDebouncer.calculate(io.limitSwitch.getAsBoolean());
    }

    public void setVoltage(double voltage) {
        if ((getAngleDegrees() > maxAngle && voltage > 0)
                || (getAngleDegrees() < minAngle && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        fieldsTable.recordOutput("voltage", voltage);
        io.setVoltage(voltage);
    }

    public void resetAngleDegrees(double angle) {
        angleDegrees.resetAngle(angle);
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Hood PID", pid);
        builder.addChild("Hood rotational helper", angleDegrees);
        builder.addDoubleProperty("Hood max angle", () -> maxAngle, (angle) -> maxAngle = angle);
        builder.addDoubleProperty("Hood min angle", () -> minAngle, (angle) -> minAngle = angle);
        DoubleHolder angleToReset = new DoubleHolder(ANGLE_OFFSET);
        builder.addDoubleProperty("Angle to reset", angleToReset::get, angleToReset::set);
        builder.addChild("Reset angle degrees", new InstantCommand(() -> {
            resetAngleDegrees(angleToReset.get());
        }).ignoringDisable(true));
    }
}
