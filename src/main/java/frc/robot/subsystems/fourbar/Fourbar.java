package frc.robot.subsystems.fourbar;

import static frc.robot.subsystems.fourbar.FourbarConstants.*;
import static frc.robot.subsystems.hood.HoodConstants.LOWER_BOUND;
import static frc.robot.subsystems.hood.HoodConstants.UPPER_BOUND;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.fourbar.io.FourbarIO;
import frc.robot.subsystems.fourbar.io.FourbarIOSim;
import frc.robot.subsystems.fourbar.io.FourbarIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Fourbar extends SubsystemBase implements Tunable {
    private Debouncer encoderConnectedDebouncer = new Debouncer(DEBOUNCER_DELAY);
    private TunableArmFeedforward feedforward = new TunableArmFeedforward(KS, KG, KV);
    private TunableTrapezoidProfile trapezoidProfile = new TunableTrapezoidProfile(
            new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private PIDController pid = new PIDController(KP, KI, KD);
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private FourbarIO io = Robot.isReal() ? new FourbarIOSparkMax(fieldsTable) : new FourbarIOSim(fieldsTable);
    private RotationalSensorHelper sensorHelper;

    private double minAngle = MIN_ANGLE;
    private double maxAngle = MAX_ANGLE;

    private double lowerBound = LOWER_BOUND;
    private double upperBound = UPPER_BOUND;

    public Fourbar() {
        sensorHelper = new RotationalSensorHelper(getAngleDegrees(), ANGLE_OFFSET);
        sensorHelper.enableContinuousWrap(lowerBound, upperBound);
    }

    public void resetPID() {
        pid.reset();
    }

    @Override
    public void periodic() {
        sensorHelper.update(io.angleDegrees.getAsDouble());
        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public double getCurrent() {
        return io.current.getAsDouble();
    }

    public boolean isEncoderConnected() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public double getAngleDegrees() {
        return sensorHelper.getAngle();
    }

    public double getVelocity() {
        return sensorHelper.getVelocity();
    }

    public void setVoltage(double voltage) {
        if ((getAngleDegrees() > maxAngle && voltage > 0)
                || (getAngleDegrees() < minAngle && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        fieldsTable.recordOutput("Desired Voltage", voltage);
    }

    public void stop() {
        io.setVolt(0);
    }

    public double calculateFeedforward(double desiredAngle, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("Desired angle", desiredAngle);
        fieldsTable.recordOutput("Desired speed", desiredSpeed);
        double volt = feedforward.calculate(desiredAngle, desiredSpeed);
        return usePID ? volt + pid.calculate(volt) : volt;

    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State desiredState) {
        return trapezoidProfile.calculate(time, initialState, desiredState);
    }

    public boolean isAtAngle(double angle) {
        return Math.abs(getAngleDegrees() - angle) < ANGLE_TOLLERANCE;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Forbar PID", pid);
        builder.addChild("Forbar FeedForward", feedforward);
        builder.addChild("Forbar TrapeziodProfile", trapezoidProfile);
        builder.addChild("Forbar RotationalSensorHelper", sensorHelper);
        builder.addDoubleProperty("Forbar minAngle", () -> minAngle, (newMinAngle) -> {
            minAngle = newMinAngle;
            sensorHelper.enableContinuousWrap(minAngle, maxAngle);
        });
        builder.addDoubleProperty("Forbar maxAngle", () -> maxAngle, (newMaxAngle) -> {
            maxAngle = newMaxAngle;
            sensorHelper.enableContinuousWrap(minAngle, maxAngle);
        });
        builder.addDoubleProperty("Forbar upper bound", () -> upperBound,
            (newUpperBound) -> {
                upperBound = newUpperBound;
                sensorHelper.enableContinuousWrap(lowerBound, newUpperBound);
        });
        builder.addDoubleProperty("Forbar lower bound", () -> lowerBound,
            (newLowerBound) -> {
                lowerBound = newLowerBound;
                sensorHelper.enableContinuousWrap(newLowerBound, upperBound);
        });
    }
}
