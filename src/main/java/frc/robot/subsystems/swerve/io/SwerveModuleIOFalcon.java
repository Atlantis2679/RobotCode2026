package frc.robot.subsystems.swerve.io;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModuleIOFalcon extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    public SwerveModuleIOFalcon(LogFieldsTable fieldsTable, int moduleNum, int driveMotorID, int turnMotorID, int canCoderID) {
        super(fieldsTable);
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);
        canCoder = new CANcoder(canCoderID);
    }

    @Override
    protected double getAbsoluteAngleRotations() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnMotor.setVoltage(voltage);
    }

    @Override
    public void setDrivePercentageSpeed(double speed) {
        driveMotor.set(speed);
    }

    @Override
    public void setTurnPercentageSpeed(double speed) {
        turnMotor.set(speed);
    }   
}
