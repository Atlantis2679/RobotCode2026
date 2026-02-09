package frc.robot.subsystems.flywheel.io;

import static frc.robot.subsystems.flywheel.FlyWheelConstants.STATOR_CURRENT_LIMIT;
import static frc.robot.subsystems.flywheel.FlyWheelConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot.subsystems.flywheel.FlyWheelConstants.SUPPLY_CURRENT_LOWER_LIMIT;
import static frc.robot.subsystems.flywheel.FlyWheelConstants.SUPPLY_CURRENT_LOWER_TIME;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.RobotMap.CANBUS;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class FlyWheelIOFalcon extends FlyWheelIO {
    private final TalonFX motor1 = new TalonFX(CANBUS.FLYWHEEL_MOTOR1_ID);
    private final TalonFX motor2 = new TalonFX(CANBUS.FLYWHEEL_MOTOR2_ID);

    private StatusCode motor1Status;
    private StatusCode motor2Status;

    public FlyWheelIOFalcon(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Flywheel");
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LOWER_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME;

        motor1Status = motor1.getConfigurator().apply(motorConfig);
        motor2Status = motor2.getConfigurator().apply(motorConfig);

        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned));

        AlertsFactory.phoenixMotor(alertsGroup, () -> motor1Status, "Motor 1");
        AlertsFactory.phoenixMotor(alertsGroup, () -> motor2Status, "Motor 2");
    }

    @Override
    public void setVoltage(double volt) {
        VoltageOut voltageOut = new VoltageOut(volt);
        motor1Status = motor1.setControl(voltageOut);
    }

    @Override
    protected double getMotorsRPM() {
        // Both motors should be running at the same speed!
        return motor1.getVelocity().getValueAsDouble() * 60 * FlyWheelConstants.GEAR_RATIO;
    }

    @Override
    protected double getMotor1Current() {
        return motor1.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    protected double getMotor2Current() {
        return motor2.getSupplyCurrent().getValueAsDouble();
    }
}
