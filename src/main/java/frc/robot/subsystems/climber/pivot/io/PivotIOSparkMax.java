package frc.robot.subsystems.climber.pivot.io;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.climber.pivot.PivotConstants.MAX_CURRENT;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap.CANBUS;
import frc.robot.RobotMap.DIO;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class PivotIOSparkMax extends PivotIO {
    private final SparkMax motor = new SparkMax(CANBUS.PIVOT_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(DIO.PIVOT_ENCODER_ID);

    public PivotIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(MAX_CURRENT);
        REVLibError configError = motor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        AlertsFactory.revMotor(new PeriodicAlertsGroup("Climber Pivot"), () -> configError, motor::getWarnings,
                motor::getFaults, "motor");
    }

    @Override
    public double getAngleDegrees() {
        return encoder.get() * 360;
    }

    @Override
    public double getPivotMotorCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public boolean getIsEncoderConnected() {
        return encoder.isConnected();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
