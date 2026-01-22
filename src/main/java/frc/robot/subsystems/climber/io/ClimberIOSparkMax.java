package frc.robot.subsystems.climber.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.climber.ClimberConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ClimberIOSparkMax extends ClimberIO{
        private SparkMax elevatorMotor = new SparkMax(CANBUS.ELEVATOR_ID, MotorType.kBrushless);
        private SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_ID, MotorType.kBrushless);
        private DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.ELEVATOR_ENCODER_ID);
    
        public ClimberIOSparkMax(LogFieldsTable fieldsTable){
            super(fieldsTable);
        }
        public double getHeightMeters(){
            return (Units.rotationsToRadians(encoder.get()) - ClimberConstants.HOMED_POSITION) * ClimberConstants.DRUM_RADIUS;;
    }
    public double getElevatorMotorCurrent(){
        return elevatorMotor.getOutputCurrent();
    }
    public double getPivotMotorCurrent(){
        return pivotMotor.getOutputCurrent();
    }
    protected boolean getIsEncoderConnected(){
        return encoder.isConnected();
    }
    public void setElevatorVoltage(double voltage){
        elevatorMotor.setVoltage(voltage);
    }
    public void setPivotVoltage(double voltage){
        pivotMotor.setVoltage(voltage);
    }
}
