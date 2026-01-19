package frc.robot.subsystems.shooter.hood.io;

import com.ctre.phoenix6.CANBus;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.*;

public class HoodIOSparkMax extends HoodIO{

    private final SparkMax hoodMotor = new SparkMax(CanBus.HOOD_MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(Dio.HOOD_ENCODER_ID);

    @Override
    public double getHoodMotorAngle() {
        return hoodEncoder;
    }

    @Override
    public void setVoltage(double volt) {

    }
    
}
