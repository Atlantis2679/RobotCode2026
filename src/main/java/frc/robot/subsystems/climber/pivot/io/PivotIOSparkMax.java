package frc.robot.subsystems.climber.pivot.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import frc.robot.RobotMap.DIO;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PivotIOSparkMax extends PivotIO{
    
    private final SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(DIO.PIVOT_ENCODER_ID);

    public PivotIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }
    @Override
    public double getPivotAngleDegrees(){
        return encoder.get() * 360;
    }

    @Override
    public double getPivotMotorCurrent(){
        return pivotMotor.getOutputCurrent();
   }

    @Override
    public boolean getIsEncoderConnected(){
        return encoder.isConnected();
    }
    
    @Override
    public void setPivotVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
    }
}
