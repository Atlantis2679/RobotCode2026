package frc.robot.subsystems.climber.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.*;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import frc.robot.RobotMap.DIO;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSparkMax extends ElevatorIO {
    private SparkMax elevatorMotor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(DIO.ELEVATOR_ENCODER_ID);

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public double getElevatorHeight(){
        return (Units.degreesToRadians(encoder.get()) - HOMED_POSITION) * DRUM_RADIUS;
    }

    public double getElevatorMotorCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    protected boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }

    public void setElevatorVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
    }
}
