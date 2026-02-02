package frc.robot.subsystems.climber.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import frc.robot.RobotMap.DIO;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.*;

public class ElevatorIOSparkMax extends ElevatorIO {
    private SparkMax motor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(DIO.ELEVATOR_ENCODER_ID);

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected double getHeightMeters() {
        return (Units.degreesToRadians(encoder.get()) - HOMED_POSITION) * DRUM_RADIUS_METERS;
    }
    
    @Override
    public double getMotorCurrent() {
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
