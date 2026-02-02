package frc.robot.subsystems.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import frc.robot.RobotMap.DIO;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class ElevatorIOSparkMax extends ElevatorIO {
    private SparkMax motor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(DIO.ELEVATOR_ENCODER_ID);

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(CURRENT_LIMIT);
        REVLibError configError = motor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(new PeriodicAlertsGroup("Elevator"), () -> configError, motor::getWarnings,
                motor::getFaults, "motor");
    }

    @Override
    protected double getHeightMeters() {
        return (Units.degreesToRadians(encoder.get()) - HOMED_POSITION) * DRUM_RADIUS_METERS;
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
