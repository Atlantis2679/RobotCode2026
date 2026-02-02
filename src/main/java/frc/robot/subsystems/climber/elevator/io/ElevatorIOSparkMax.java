package frc.robot.subsystems.climber.elevator.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.*;

import com.revrobotics.spark.SparkMax;

<<<<<<< HEAD:src/main/java/frc/robot/subsystems/climber/elevator/io/ElevatorIOSparkMax.java
=======
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

>>>>>>> 9ce40bc287b65772dfab444b3bf35fb7c4d0a362:src/main/java/frc/robot/subsystems/elevator/io/ElevatorIOSparkMax.java
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
