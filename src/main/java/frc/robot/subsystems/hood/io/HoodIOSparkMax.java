package frc.robot.subsystems.hood.io;

import com.revrobotics.spark.SparkMax;

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
    private REVLibError configError;

    public HoodIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        config.idleMode(IdleMode.kBrake);
        configError = motor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance.getSubGroup("Hood"), () -> configError,
                motor::getWarnings, motor::getFaults,
                "motor");
    }

    @Override
    public double getMotorRotations() {
        return -motor.getEncoder().getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(-voltage);
    }

    @Override
    public void setCoast() {
        config.idleMode(IdleMode.kCoast);
    }

    @Override
    protected double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    protected double getAbsolueAngleDegrees() {
        return motor.getAbsoluteEncoder().getPosition() * 360 * GEAR_RATIO;
    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        config.smartCurrentLimit((int)currentLimit);
        configError = motor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
}
