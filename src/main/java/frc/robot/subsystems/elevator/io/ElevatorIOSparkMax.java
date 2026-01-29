package frc.robot.subsystems.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSparkMax extends ElevatorIO {
    private SparkMax elevatorMotor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.ELEVATOR_ENCODER_ID);

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public double getEncoderAngleDegrees(){
        return encoder.get();
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
