package frc.robot.subsystems.climber.pivot;

import static frc.robot.subsystems.climber.pivot.PivotConstants.*;
import static frc.robot.subsystems.climber.pivot.PivotConstants.Sim.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.climber.pivot.io.PivotIO;
import frc.robot.subsystems.climber.pivot.io.PivotIOSim;
import frc.robot.subsystems.climber.pivot.io.PivotIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Pivot extends SubsystemBase{
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final PivotIO io = Robot.isReal() ? new PivotIOSparkMax(fieldsTable) : new PivotIOSim(fieldsTable);

        private final RotationalSensorHelper rotationalSensorHelper;

        private final TunableTrapezoidProfile pivotTrapezoid = new TunableTrapezoidProfile(
        new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC,
                    MAX_ACCELERATION_DEG_PER_SEC_SQUEARD));

    private PIDController pivotPIDController = new PIDController(KP, KI, KD);
    
    private TunableArmFeedforward pivotFeedforward = Robot.isReal()
            ? new TunableArmFeedforward(KS, KG,
                    KV, KA)
            : new TunableArmFeedforward(SIM_KS, SIM_KG,
                    SIM_KV, SIM_KA);
            
    private final Debouncer encoderConnectedDebouncer = new Debouncer(
            ENCODER_CONNECTED_DEBAUNCER_SEC);
    
    private double maxAngle = MAX_ANGLE_DEGREES;
    private double minAngle = MIN_ANGLE_DEGREES;

    private PivotVisualizer realVisualizer = new PivotVisualizer(fieldsTable, "Real Visualizer",
            new Color8Bit(Color.kPurple));
    private final PivotVisualizer desiredVisualizer = new PivotVisualizer(fieldsTable, "Desired Visualizer",
            new Color8Bit(Color.kYellow));
    public Pivot(){
        fieldsTable.update();
    
        TunablesManager.add("Pivot", (Tunable) this);

        rotationalSensorHelper = new RotationalSensorHelper(io.pivotAngleDegrees.getAsDouble(), ANGLE_OFFSET); 
    }
    
    @Override
    public void periodic() {
        fieldsTable.recordOutput("Current command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        fieldsTable.recordOutput("Pivot Angle", getAngleDegrees());
        fieldsTable.recordOutput("Pivot Motor Current", io.pivotMotorCurrect.getAsDouble());
        fieldsTable.recordOutput("Pivot Connected Debouncer", getEncoderConnectedDebouncer());

        realVisualizer.update(getAngleDegrees());
    }

    public boolean getEncoderConnectedDebouncer() {
        return encoderConnectedDebouncer.calculate(io.isEncoderConnected.getAsBoolean());
    }

    public void setPivotVoltage(double voltage) {
        if ((getAngleDegrees() > maxAngle && voltage > 0)
                || (getAngleDegrees() < minAngle && voltage < 0)) {
            voltage = 0.0;
        }
        voltage = MathUtil.clamp(voltage, -MAX_VOLTAGE,
                MAX_VOLTAGE);
        fieldsTable.recordOutput("Elevator Voltage", voltage);
        io.setPivotVoltage(voltage);
    }

    public double getAngleDegrees() {
        return io.pivotAngleDegrees.getAsDouble();
    }

    public double getAngularVelocity() {
        return rotationalSensorHelper.getVelocity();
    }

    public double calculateFeedForward(double desiredAngle, double desiredSpeed, boolean usePID) {
        fieldsTable.recordOutput("Desired Angle", desiredAngle);
        fieldsTable.recordOutput("Desired Speed", desiredSpeed);
        desiredVisualizer.update(desiredAngle);
        double speed = pivotFeedforward.calculate(desiredAngle, desiredSpeed);
        if (usePID && !isAtAngle(desiredAngle)) {
            speed += pivotPIDController.calculate(getAngleDegrees(), desiredAngle);
        }
        return speed;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
            TrapezoidProfile.State goalState) {
        return pivotTrapezoid.calculate(time, initialState, goalState);
    }

    public boolean isAtAngle(double desiredAngle) {
        return Math.abs(desiredAngle - getAngleDegrees()) <= ANGLE_TOLERENCE;
    }

    public void resetPID() {
        pivotPIDController.reset();
    }

    public void stop() {
        io.setPivotVoltage(0);
    }

    public void initTunable(TunableBuilder builder) {
        builder.addChild("Pivot PID", pivotPIDController);
        builder.addChild("Pivot feedforward", pivotFeedforward);
        builder.addChild("Pivot Trapezoid profile", pivotTrapezoid);
        builder.addChild("Pivot RotationalSensorHelper", rotationalSensorHelper);
        builder.addDoubleProperty("Pivot max angle", () -> maxAngle, (height) -> maxAngle = height);
        builder.addDoubleProperty("Pivot min angle", () -> minAngle, (height) -> minAngle = height);
    }
}

