package frc.robot.subsystems.flywheel.io;

import static frc.robot.subsystems.flywheel.FlyWheelConstants.SUPPLY_CURRENT_LOWER_LIMIT;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotMap.CANBUS;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class FlyWheelIOSparkMax extends FlyWheelIO {
    private final SparkMax motor1 = new SparkMax(CANBUS.FLYWHEEL_MOTOR1_ID, MotorType.kBrushless);
    private final SparkMax motor2 = new SparkMax(CANBUS.FLYWHEEL_MOTOR2_ID, MotorType.kBrushless);

    public FlyWheelIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Flywheel");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit((int)SUPPLY_CURRENT_LOWER_LIMIT);

        REVLibError motor1ConfigError = motor1.configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alertsGroup, () -> motor1ConfigError, motor1::getWarnings, motor1::getFaults, "motor1");
        REVLibError motor2ConfigError = motor1.configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(alertsGroup, () -> motor2ConfigError, motor2::getWarnings, motor2::getFaults, "motor2");

    }

    @Override
    public void setVoltage(double volt) {
        motor1.setVoltage(volt);
        motor2.setVoltage(volt);
    }

    @Override
    protected double getMotorsRPM() {
        // Both motors should be running at the same speed!
        return motor1.getAbsoluteEncoder().getVelocity() * FlyWheelConstants.GEAR_RATIO;
    }

    @Override
    protected double getMotor1Current() {
        return motor1.getOutputCurrent();
    }

    @Override
    protected double getMotor2Current() {
        return motor2.getOutputCurrent();
    }
}
