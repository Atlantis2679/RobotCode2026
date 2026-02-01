package frc.robot.subsystems.intake.roller.io;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotMap;
import frc.robot.subsystems.intake.roller.RollerConstants;
import frc.robot.utils.AlertsFactory;
import frc.robot.RobotMap.CANBUS;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class RollerIOSparkMax extends RollerIO{
    
    private SparkMax motor = new SparkMax(RobotMap.CANBUS.ROLLER_ID, MotorType.kBrushless);

    public RollerIOSparkMax(LogFieldsTable fields){
        super(fields);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(RollerConstants.CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kBrake);
        REVLibError motorConfigError = motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance, 
            () -> motorConfigError, motor::getWarnings, motor::getFaults, "Roller Config");
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }
    protected double getCurrent(){
        return motor.getOutputCurrent();
    }

}
