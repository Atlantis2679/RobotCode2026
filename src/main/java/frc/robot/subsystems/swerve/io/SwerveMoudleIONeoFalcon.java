package frc.robot.subsystems.swerve.io;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.AlertsFactory;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class SwerveMoudleIONeoFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final SparkMax turnMotor;
    private final CANcoder canCoder;

    private final VoltageOut driveVoltageControl = new VoltageOut(0);
    private final DutyCycleOut drivePercentageControl = new DutyCycleOut(0);

    private StatusCode driveMotorStatus;
    private REVLibError turnMotorStatus;
    private StatusCode canCoderStatus;

    SparkMaxConfig turnMotorConfig;

    public SwerveMoudleIONeoFalcon(LogFieldsTable fieldsTable, int moduleNum, int driveMotorID, int turnMotorID,
            int canCoderID) {
        super(fieldsTable);
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);
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

        turnMotorConfig = new SparkMaxConfig();

        turnMotorConfig.idleMode(IdleMode.kBrake);
        turnMotorConfig.encoder.positionConversionFactor(TURN_GEAR_RATIO);
        turnMotorConfig.closedLoop.positionWrappingEnabled(true);
        turnMotorConfig.closedLoop.positionWrappingInputRange(-0.5, 0.5);
        turnMotorConfig.smartCurrentLimit((int) TURN_STATOR_CURRENT_LIMIT);
        turnMotorConfig.closedLoop.pid(TURN_MOTOR_KP, TURN_MOTOR_KI, TURN_MOTOR_KD);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        driveMotorStatus = driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotorStatus = turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        canCoderStatus = canCoder.getConfigurator().apply(canCoderConfig);

        turnMotor.getEncoder().setPosition(0);

        String moduleAlertPrefix = "Module " + moduleNum + " " + getModuleName(moduleNum) + " ";

        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> driveMotorStatus, moduleAlertPrefix + "Drive Motor Status");
        AlertsFactory.revMotor(PeriodicAlertsGroup.defaultInstance,
                () -> turnMotorStatus, turnMotor::getWarnings, turnMotor::getFaults,
                moduleAlertPrefix + "Turn Motor Status");
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> canCoderStatus, moduleAlertPrefix + "Can Coder Status");
    }

    @Override
    protected double getAbsoluteTurnAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }
    
    @Override
    protected double getIntegratedTurnAngleRotations() {
        return turnMotor.getEncoder().getPosition();
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
        turnMotor.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
    }

    @Override
    protected double getDriveDistanceRotations() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setCoast() {
        driveMotor.setControl(new CoastOut());
        // turnMotorConfig.idleMode(IdleMode.kCoast);
        // turnMotorStatus = turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters,
        //         PersistMode.kNoPersistParameters);
    }

    @Override
    public void resetIntegratedAngleRotations(double newAngle) {
        turnMotor.getEncoder().setPosition(newAngle);
    }

    @Override
    protected double getTurnKP() {
        return turnMotor.configAccessor.closedLoop.getP();
    }

    @Override
    protected double getTurnKI() {
        return turnMotor.configAccessor.closedLoop.getI();
    }

    @Override
    protected double getTurnKD() {
        return turnMotor.configAccessor.closedLoop.getD();
    }

    @Override
    public void setTurnKP(double kP) {
        turnMotorConfig.closedLoop.p(kP);
        turnMotorStatus = turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTurnKI(double kI) {
        turnMotorConfig.closedLoop.i(kI);
        turnMotorStatus = turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void setTurnKD(double kD) {
        turnMotorConfig.closedLoop.d(kD);
        turnMotorStatus = turnMotor.configure(turnMotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
}
