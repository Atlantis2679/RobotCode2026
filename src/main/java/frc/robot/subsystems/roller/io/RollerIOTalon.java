package frc.robot.subsystems.roller.io;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.RobotMap;
import static frc.robot.subsystems.roller.RollerConstants.*;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class RollerIOTalon extends RollerIO {
    private TalonFX motor = new TalonFX(RobotMap.CANBUS.ROLLER_ID);
    private StatusCode statusCode;

    public RollerIOTalon(LogFieldsTable fields) {
        super(fields);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        statusCode = motor.getConfigurator().apply(motorConfig);
        PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Roller");
        AlertsFactory.phoenixMotor(alertsGroup, () -> statusCode, "Roller error");
    }

    public void setVoltage(double voltage) {
        statusCode = motor.setControl(new VoltageOut(-voltage));
    }

    protected double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

}
