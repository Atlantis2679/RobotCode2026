package frc.robot.subsystems.fourbar.io;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import static frc.robot.subsystems.fourbar.FourbarConstants.*;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

import static frc.robot.RobotMap.DIO.FOURBAR_ENCODER_ID;

public class FourbarIOTalon extends FourbarIO {

    private TalonFX motor = new TalonFX(RobotMap.CANBUS.FOURBAR_ID);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(FOURBAR_ENCODER_ID);
    private StatusCode motorStatusCode;

    public FourbarIOTalon(LogFieldsTable fields) {
        super(fields);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorStatusCode = motor.getConfigurator().apply(motorConfig);
        PeriodicAlertsGroup periodicAlertsGroup = new PeriodicAlertsGroup("Roller");
        AlertsFactory.phoenixMotor(periodicAlertsGroup, () -> motorStatusCode, "Roller errors");
    }

    @Override
    protected double getAngleDegrees() {
        return encoder.get() * 360;
    }

    protected double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void setVolt(double volt) {
        motorStatusCode = motor.setControl(new VoltageOut(volt));
    }
}
