package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.io.ClimberIO;
import frc.robot.subsystems.climber.io.ClimberIOSim;
import frc.robot.subsystems.climber.io.ClimberIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Climber extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final ClimberIO io = Robot.isReal() ? new ClimberIOSparkMax(fieldsTable) : new ClimberIOSim(fieldsTable);

    private final TunableTrapezoidProfile elevatorTrapezoid = new TunableTrapezoidProfile(
            new TrapezoidProfile.Constraints(ClimberConstants.Elevator.MAX_VELOCITY_METERS_PER_SEC,
                    ClimberConstants.Elevator.MAX_ACCELERATION_METER_PER_SEC_SQUARED));

    private PIDController elevatorPidController = new PIDController(
            ClimberConstants.Elevator.KP,
            ClimberConstants.Elevator.KI,
            ClimberConstants.Elevator.KD);

    private TunableArmFeedforward elevatorFeedforward = Robot.isReal()
            ? new TunableArmFeedforward(ClimberConstants.Elevator.KS, ClimberConstants.Elevator.KG,
                    ClimberConstants.Elevator.KV)
            : new TunableArmFeedforward(ClimberConstants.Elevator.Sim.SIM_KS, ClimberConstants.Elevator.Sim.SIM_KG,
                    ClimberConstants.Elevator.Sim.SIM_KV, ClimberConstants.Elevator.Sim.SIM_KA);

    private final Debouncer encoderConnectedDebouncer = new Debouncer(
            ClimberConstants.Elevator.ENCODER_CONNECTED_DEBAUNCER_SEC);

    private final RotationalSensorHelper elevatorRotationalSensorHelper;

    public Climber() {
        fieldsTable.update();

        elevatorRotationalSensorHelper = new RotationalSensorHelper(io.getEncoderAngleDegrees(),
                ClimberConstants.Elevator.ANGLE_OFFSET);
    }

    @Override
    public void periodic() {
        elevatorRotationalSensorHelper.update(io.getEncoderAngleDegrees());

        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        fieldsTable.recordOutput("Elevator Height", io.getHeightMeters());
        fieldsTable.recordOutput("Elevator motor current", io.getElevatorMotorCurrent());
        fieldsTable.recordOutput("Pivot motor current", io.getPivotMotorCurrent());
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public void setElevatorVoltage(double voltage) {
        if ((getHeight() > ClimberConstants.Elevator.MAX_HEIGHT_METERS && voltage > 0)
                || (getHeight() < ClimberConstants.Elevator.MIN_HEIGHT_METERS && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -ClimberConstants.Elevator.MAX_VOLTAGE,
                ClimberConstants.Elevator.MAX_VOLTAGE);
        fieldsTable.recordOutput("Elevator Voltage", voltage);
        io.setElevatorVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -ClimberConstants.Pivot.MAX_VOLTAGE,
                ClimberConstants.Pivot.MAX_VOLTAGE);
        fieldsTable.recordOutput("Pivot Voltage", voltage);
        io.setPivotVoltage(voltage);
    }

    public double getHeight() {
        return io.getHeightMeters();
    }

    public double getAngularVelocity() {
        return elevatorRotationalSensorHelper.getVelocity();
    }

    public double getHeightVelocity() {
        return Units.degreesToRadians(getAngularVelocity())
                * ClimberConstants.Elevator.DRUM_RADIUS;
    }

    public double calculateFeedForward(double desiredHeight, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("Desired  Height", desiredHeight);
        fieldsTable.recordOutput("Desired Speed", desiredSpeed);
        double speed = elevatorFeedforward.calculate(desiredHeight, desiredSpeed);
        if (usePID && !isAtHeight(desiredHeight)) {
            speed += elevatorPidController.calculate(getHeight(), desiredHeight);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return elevatorTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtHeight(double desiredHeight) {
        return Math.abs(desiredHeight - getHeight()) <= ClimberConstants.Elevator.HEIGHT_TOLERENCE;
    }

    public void resetPID() {
        elevatorPidController.reset();
    }

    public void stop() {
        io.setElevatorVoltage(0);
        io.setPivotVoltage(0);
    }
}
