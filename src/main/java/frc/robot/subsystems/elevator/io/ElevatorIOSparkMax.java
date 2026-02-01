package frc.robot.subsystems.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSparkMax extends ElevatorIO {
    private SparkMax elevatorMotor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.ELEVATOR_ENCODER_ID);

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected double getElevatorHeight() {
        return (Units.degreesToRadians(encoder.get()) - HOMED_POSITION) * DRUM_RADIUS;
    }
    
    @Override
    public double getElevatorMotorCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
    }
}
