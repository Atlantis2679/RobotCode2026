package frc.robot.subsystems.hood.io;

import com.revrobotics.spark.SparkMax;

import static frc.robot.subsystems.hood.HoodConstants.CURRENT_LIMIT;
import static frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.RobotMap.*;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class HoodIOSparkMax extends HoodIO {
    private final SparkMax motor = new SparkMax(CANBUS.HOOD_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxConfig config = new SparkMaxConfig();

    public HoodIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        config.smartCurrentLimit(CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        REVLibError configError = motor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance.getSubGroup("Hood"), () -> configError, motor::getWarnings, motor::getFaults,
                "motor");
    }

    @Override
    public double getHoodMotorAngleDegree() {
        return motor.getEncoder().getPosition() * GEAR_RATIO;
    }

    @Override
    public void setVoltage(double volt) {
        motor.setVoltage(volt);
    }

    @Override
    public void setCoast() {
        config.idleMode(IdleMode.kCoast);
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return true;
    }
}
