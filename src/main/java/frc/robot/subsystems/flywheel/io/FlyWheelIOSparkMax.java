package frc.robot.subsystems.flywheel.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class FlyWheelIOSparkMax extends FlyWheelIO{

    private final SparkMax motor1 = new SparkMax(CANBUS.FLYWHEEL_MOTOR1_ID, MotorType.kBrushless);
    private final SparkMax motor2 = new SparkMax(CANBUS.FLYWHEEL_MOTOR2_ID, MotorType.kBrushless);


    public FlyWheelIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }


    @Override
    public double getAbsoluteRotations() {
        return motor1.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void setVoltage(double volt) {
        motor1.setVoltage(volt);
        motor2.setVoltage(volt);
    }
    
}
