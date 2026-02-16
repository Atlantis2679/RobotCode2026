package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSim;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;

import static frc.robot.subsystems.elevator.ElevatorConstants.Sim.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Elevator extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final ElevatorIO io = Robot.isReal() ? new ElevatorIOSparkMax(fieldsTable) : new ElevatorIOSim(fieldsTable);

    private final ElevatorVisualizer realVisualizer = new ElevatorVisualizer(fieldsTable, "Real Visualizer",
            new Color8Bit(Color.kPurple));
    private final ElevatorVisualizer desiredVisualizer = new ElevatorVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));

    private final TunableTrapezoidProfile trapezoidProfile = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY_METERS_PER_SEC,
                    MAX_ACCELERATION_METER_PER_SEC_SQUARED));

    private PIDController pid = new PIDController(KP, KI, KD);

    private TunableArmFeedforward elevatorFeedforward = Robot.isReal()
            ? new TunableArmFeedforward(KS, KG,
                    KV, KA)
            : new TunableArmFeedforward(SIM_KS, SIM_KG,
                    SIM_KV, SIM_KA);

    private final Debouncer encoderConnectedDebouncer = new Debouncer(
            ENCODER_CONNECTED_DEBAUNCER_SEC);

    private double maxHeight = MAX_HEIGHT_METERS;
    private double minHeight = MIN_HEIGHT_METERS;

    private double previousHeight;
    private double currTimeSec;
    private double prevTimeSec;

    public Elevator() {
        fieldsTable.update();

        TunablesManager.add("Elevator", (Tunable) this);
    }

    @Override
    public void periodic() {
        realVisualizer.update(getHeightMeters());

        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        fieldsTable.recordOutput("Elevator Height", getHeightMeters());
        fieldsTable.recordOutput("Elevator Motor Current", io.motorCurrent.getAsDouble());
        fieldsTable.recordOutput("Encoder Connected Debouncer", getEncoderConnectedDebouncer());

        prevTimeSec = currTimeSec;
        currTimeSec = Timer.getFPGATimestamp();
        previousHeight = getHeightMeters();
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public void setElevatorVoltage(double voltage) {
        if ((getHeightMeters() > maxHeight && voltage > 0)
                || (getHeightMeters() < minHeight && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE,
                MAX_VOLTAGE);
        fieldsTable.recordOutput("Elevator Voltage", voltage);
        io.setVoltage(voltage);
    }

    public double getHeightMeters() {
        return io.heightMeters.getAsDouble();
    }

    public double getHeightVelocity() {
        double deltaTime = currTimeSec - prevTimeSec;
        if (deltaTime == 0) {
            DriverStation.reportWarning(
                    "You should not request velocity after no time passed (probably called in initial loop).", true);
            return 0;
        }
        return (getHeightMeters() - previousHeight) / deltaTime;
    }

    public double calculateFeedForward(double desiredHeight, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("Desired Height", desiredHeight);
        fieldsTable.recordOutput("Desired Speed", desiredSpeed);
        desiredVisualizer.update(desiredHeight);
        double speed = elevatorFeedforward.calculate(desiredHeight, desiredSpeed);
        if (usePID && !isAtHeight(desiredHeight)) {
            speed += pid.calculate(getHeightMeters(), desiredHeight);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return trapezoidProfile.calculate(time, initialState, goalState);
    }

    public boolean isAtHeight(double desiredHeight) {
        return Math.abs(desiredHeight - getHeightMeters()) <= HEIGHT_TOLERENCE;
    }

    public void resetPID() {
        pid.reset();
    }

    public void stop() {
        io.setVoltage(0);
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Elevator PID", pid);
        builder.addChild("Elevator feedforward", elevatorFeedforward);
        builder.addChild("Elevator Trapezoid profile", trapezoidProfile);
        builder.addDoubleProperty("Elevator max height", () -> maxHeight, (height) -> maxHeight = height);
        builder.addDoubleProperty("Elevator min height", () -> minHeight, (height) -> minHeight = height);
    }
}
