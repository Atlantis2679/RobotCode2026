package frc.robot.subsystems.fourbar;

import static frc.robot.subsystems.fourbar.FourbarConstants.*;

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

    public Fourbar() {
        sensorHelper = new RotationalSensorHelper(getAngleDegrees(), ANGLE_OFFSET);
        sensorHelper.enableContinuousWrap(MIN_ANGLE, MAX_ANGLE);
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

    public void setVoltage(double volt) {
        volt = MathUtil.clamp(volt, -MAX_VOLTAGE, MAX_VOLTAGE);
        fieldsTable.recordOutput("Desired Voltage", volt);
    }

    public void stop() {
        io.setVolt(0);
    }

    public double calculateFeedforward(double desiredAngle, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("desired angle", desiredAngle);
        fieldsTable.recordOutput("desired speed", desiredSpeed);
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
        builder.addChild("PID", pid);
        builder.addChild("FeedForward", feedforward);
        builder.addChild("TrapeziodProfile", trapezoidProfile);
        builder.addChild("RotationalSensorHelper", sensorHelper);
        builder.addDoubleProperty("minAngle", () -> minAngle, (newMinAngle) -> {
            minAngle = newMinAngle;
            sensorHelper.enableContinuousWrap(minAngle, maxAngle);
        });
        builder.addDoubleProperty("maxAngle", () -> maxAngle, (newMaxAngle) -> {
            maxAngle = newMaxAngle;
            sensorHelper.enableContinuousWrap(minAngle, maxAngle);
        });
    }
}
