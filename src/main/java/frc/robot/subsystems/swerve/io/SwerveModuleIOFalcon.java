package frc.robot.subsystems.swerve.io;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    private final VoltageOut driveVoltageControl = new VoltageOut(0);
    private final DutyCycleOut drivePercentageControl = new DutyCycleOut(0);

    private final PositionVoltage turnVoltageControl = new PositionVoltage(0).withSlot(0);

    private StatusCode driveMotorStatus;
    private StatusCode turnMotorStatus;
    private StatusCode canCoderStatus;

    private final Slot0Configs turnSlotConfigs;

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int moduleNum, int driveMotorID, int turnMotorID,
            int canCoderID) {
        super(fieldsTable);
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANcoder(canCoderID);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;

        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = DRIVE_SUPPLY_CURRENT_LOWER_LIMIT;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = DRIVE_SUPPLY_CURRENT_LOWER_TIME;

        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfig.CurrentLimits.StatorCurrentLimit = TURN_STATOR_CURRENT_LIMIT;

        turnSlotConfigs = turnMotorConfig.Slot0;
        turnSlotConfigs.kP = TURN_MOTOR_KP;
        turnSlotConfigs.kI = TURN_MOTOR_KI;
        turnSlotConfigs.kD = TURN_MOTOR_KD;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        driveMotorStatus = driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotorStatus = turnMotor.getConfigurator().apply(turnMotorConfig);
        canCoderStatus = canCoder.getConfigurator().apply(canCoderConfig);
        
        turnMotor.setPosition(0);

        String moduleAlertPrefix = "Module " + moduleNum + " " + getModuleName(moduleNum) + " ";

        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> driveMotorStatus, moduleAlertPrefix + "Drive Motor Status");
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> turnMotorStatus, moduleAlertPrefix + "Turn Motor Status");
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> canCoderStatus, moduleAlertPrefix + "Can Coder Status");
    }

    @Override
    protected double getAbsoluteTurnAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }
    
    @Override
    protected double getIntegratedTurnAngleRotations() {
        return turnMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setControl(driveVoltageControl.withOutput(voltage));
    }

    @Override
    public void setDrivePercentageSpeed(double speed) {
        driveMotor.setControl(drivePercentageControl.withOutput(speed));
    }

    @Override
    public void setTurnAngleRotations(double rotations) {
        turnMotor.setControl(turnVoltageControl.withPosition(rotations));
    }

    @Override
    protected double getDriveDistanceRotations() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getDriveSpeedRPS() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setCoast() {
        driveMotor.setControl(new CoastOut());
        turnMotor.setControl(new CoastOut());
    }

    @Override
    public void resetIntegratedAngleRotations(double newAngle) {
        turnMotor.setPosition(newAngle);
    }

    @Override
    protected double getTurnKP() {
        return turnSlotConfigs.kP;
    }

    @Override
    protected double getTurnKI() {
        return turnSlotConfigs.kI;
    }

    @Override
    protected double getTurnKD() {
        return turnSlotConfigs.kD;
    }

    @Override
    public void setTurnKP(double kP) {
        turnSlotConfigs.kP = kP;
    }

    @Override
    public void setTurnKI(double kI) {
        turnSlotConfigs.kI = kI;
    }

    @Override
    public void setTurnKD(double kD) {
        turnSlotConfigs.kD = kD;
    }

    @Override
    protected double getCurrent() {
        return driveMotor.getTorqueCurrent().getValueAsDouble();
    }
}
