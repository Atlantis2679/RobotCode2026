package frc.robot.subsystems.intake.forbar.io;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.forbar.ForbarConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class SlapdownIOSparkMax extends SlapdownIO {

    private SparkMax motor = new SparkMax(RobotMap.CANBUS.SLAPDOWN_ID, MotorType.kBrushless);
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private DutyCycleEncoder encoder = new DutyCycleEncoder(RobotMap.DIO.SLAPDOWN_ENCODER_ID);

    public SlapdownIOSparkMax(LogFieldsTable fields) {
        super(fields);

        motorConfig.smartCurrentLimit(ForbarConstants.CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kCoast);
        REVLibError motorConfigError = motor.configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance,
                () -> motorConfigError, motor::getWarnings, motor::getFaults, "Slapdown motor");

        encoder.setDutyCycleRange(0, 1);
    }

    // input:
    protected double getAngleDegrees() {
        return encoder.get();
    }

    protected boolean isEncoderConnected() {
        return encoder.isConnected();
    }

    protected double getCurrent() {
        return motor.getOutputCurrent();
    }

    // output:
    public void setVolt(double volt) {
        motor.setVoltage(volt);
    }
}
