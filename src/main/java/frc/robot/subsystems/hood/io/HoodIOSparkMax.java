package frc.robot.subsystems.hood.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.*;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class HoodIOSparkMax extends HoodIO{

    private final SparkMax hoodMotor = new SparkMax(CANBUS.HOOD_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(DIO.HOOD_ENCODER_ID);

    public HoodIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    @Override
    public double getHoodMotorAngle() {
        return hoodEncoder.get() * 360;
    }

    @Override
    public void setVoltage(double volt) {
        hoodMotor.setVoltage(volt);
    }
    @Override
    protected boolean getIsEncoderConnected() {
        return hoodEncoder.isConnected();
    }
    
}
