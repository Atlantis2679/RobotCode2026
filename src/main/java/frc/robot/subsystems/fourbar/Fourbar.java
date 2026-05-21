package frc.robot.subsystems.fourbar;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.fourbar.FourbarConstants.*;
import static frc.robot.subsystems.hood.HoodConstants.MAX_ANGLE_DEGREES;
import static frc.robot.subsystems.hood.HoodConstants.MIN_ANGLE_DEGREES;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.fourbar.io.FourbarIO;
import frc.robot.subsystems.fourbar.io.FourbarIOSim;
import frc.robot.subsystems.fourbar.io.FourbarIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Fourbar extends SubsystemBase implements Tunable {
    private final PIDController pid = new PIDController(KP, KI, KD);
    private final TunableArmFeedforward feedforward = new TunableArmFeedforward(KS, KG, KV);
    private final TunableTrapezoidProfile trapezoidProfile = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC));
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final FourbarIO io = Robot.isReal() ? new FourbarIOSparkMax(fieldsTable) : new FourbarIOSim(fieldsTable);
    private final RotationalSensorHelper angleDegrees = new RotationalSensorHelper(io.angleDegrees.getAsDouble());

    private final Debouncer isStuckDebouncer = new Debouncer(STUCK_DEBOUNCE_SEC, DebounceType.kRising);

    private double desiredVoltage = 0;

    public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((volt) -> this.setVoltage(volt.in(Volts), false), log -> {
                log.motor("fourbar-motor")
                        .voltage(Voltage.ofBaseUnits(desiredVoltage, Volts))
                        .angularPosition(Angle.ofBaseUnits(getAngleDegrees(), Degrees))
                        .angularVelocity(AngularVelocity.ofBaseUnits(getVelocity(), DegreesPerSecond));
            }, this));

    public Fourbar() {
        TunablesManager.add(getName(), (Tunable) this);
        pid.enableContinuousInput(0, 360);
    }

    public void resetPID() {
        pid.reset();
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

    public void setVoltage(double voltage, boolean softwareStop) {
        if (softwareStop &&
            ((getAngleDegrees() > MAX_ANGLE_DEGREES && voltage > 0) ||
            (getAngleDegrees() < MIN_ANGLE_DEGREES && voltage < 0))) {
            voltage = 0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        desiredVoltage = voltage;
        io.setVolt(voltage);
    }

    public double calculateFeedForward(double desiredAngleDegrees, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("desired angle", desiredAngleDegrees);
        fieldsTable.recordOutput("desired speed", desiredSpeed);
        double speed = feedforward.calculate(Math.toRadians(desiredAngleDegrees), desiredSpeed);
        if (usePID && !isAtAngle(desiredAngleDegrees)) {
            speed += pid.calculate(getAngleDegrees(), desiredAngleDegrees);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return trapezoidProfile.calculate(time, initialState, goalState);
    }

    public void stop() {
        desiredVoltage = 0;
        io.setVolt(0);
    }

    public boolean isAtAngle(double angle) {
        return Math.abs(getAngleDegrees() - angle) < ANGLE_TOLLERANCE;
    }

    public boolean isStuck() {
        return isStuckDebouncer.calculate(
                Math.abs(desiredVoltage) > 0 && Math.abs(getVelocity()) < STUCK_VELOCITY_THRESHOLD_DEG_PER_SEC);
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Forbar PID", pid);
        builder.addChild("Forbar FeedForward", feedforward);
        builder.addChild("Forbar TrapeziodProfile", trapezoidProfile);
        builder.addChild("Forbar RotationalSensorHelper", angleDegrees);
    }
}
