package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.io.ClimberIO;
import frc.robot.subsystems.climber.io.ClimberIOSim;
import frc.robot.subsystems.climber.io.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberConstants.*;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Climber extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final ClimberIO io = Robot.isReal() ? new ClimberIOSparkMax(fieldsTable) : new ClimberIOSim(fieldsTable);

    private final ClimberVisualizer realVisualizer = new ClimberVisualizer(fieldsTable, "Real Visualizer",
        new Color8Bit(Color.kPurple));
    private final ClimberVisualizer desiredHoodVisualizer = new ClimberVisualizer(fieldsTable, "Desired Visualizer",
        new Color8Bit(Color.kYellow));

    private final TunableTrapezoidProfile elevatorTrapezoid = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(Elevator.MAX_VELOCITY_METERS_PER_SEC,
                    Elevator.MAX_ACCELERATION_METER_PER_SEC_SQUARED));

    private PIDController elevatorPidController = new PIDController(
            Elevator.KP,
            Elevator.KI,
            Elevator.KD);

    private TunableArmFeedforward elevatorFeedforward = Robot.isReal()
            ? new TunableArmFeedforward(Elevator.KS, Elevator.KG,
                    Elevator.KV, Elevator.KA)
            : new TunableArmFeedforward(Elevator.Sim.SIM_KS, Elevator.Sim.SIM_KG,
                    Elevator.Sim.SIM_KV, Elevator.Sim.SIM_KA);

    private final Debouncer encoderConnectedDebouncer = new Debouncer(
            Elevator.ENCODER_CONNECTED_DEBAUNCER_SEC);

    private final RotationalSensorHelper elevatorRotationalSensorHelper;

    public Climber() {
        fieldsTable.update();

        elevatorRotationalSensorHelper = new RotationalSensorHelper(io.getEncoderAngleDegrees(),
                Elevator.ANGLE_OFFSET);
    }

    @Override
    public void periodic() {
        realVisualizer.update(getHeightMeters());
        elevatorRotationalSensorHelper.update(getAngleDegrees());

        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        fieldsTable.recordOutput("Elevator Height", getHeightMeters());
        fieldsTable.recordOutput("Elevator motor current", io.getElevatorMotorCurrent());
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }
    
    public void setElevatorVoltage(double voltage) {
        if ((getHeightMeters() > Elevator.MAX_HEIGHT_METERS && voltage > 0)
                || (getHeightMeters() < Elevator.MIN_HEIGHT_METERS && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -Elevator.MAX_VOLTAGE,
                Elevator.MAX_VOLTAGE);
        fieldsTable.recordOutput("Elevator Voltage", voltage);
        io.setElevatorVoltage(voltage);
    }

    public double getAngleDegrees(){
        return io.getEncoderAngleDegrees();
    }

    public double getHeightMeters() {
        return (Units.degreesToRadians(io.getEncoderAngleDegrees()) - ClimberConstants.Elevator.HOMED_POSITION)
                * ClimberConstants.Elevator.DRUM_RADIUS;
    }

    public double getAngularVelocity() {
        return elevatorRotationalSensorHelper.getVelocity();
    }

    public double getHeightVelocity() {
        return Units.degreesToRadians(getAngularVelocity())
                * Elevator.DRUM_RADIUS;
    }

    public double calculateFeedForward(double desiredHeight, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("Desired Height", desiredHeight);
        fieldsTable.recordOutput("Desired Speed", desiredSpeed);
        desiredHoodVisualizer.update(desiredHeight);
        double speed = elevatorFeedforward.calculate(desiredHeight, desiredSpeed);
        if (usePID && !isAtHeight(desiredHeight)) {
            speed += elevatorPidController.calculate(getHeightMeters(), desiredHeight);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return elevatorTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtHeight(double desiredHeight) {
        return Math.abs(desiredHeight - getHeightMeters()) <= Elevator.HEIGHT_TOLERENCE;
    }

    public void resetPID() {
        elevatorPidController.reset();
    }

    public void stop() {
        io.setElevatorVoltage(0);
    }
}
