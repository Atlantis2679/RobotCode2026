package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_VOLTAGE;
import static frc.robot.subsystems.swerve.SwerveConstants.Sim.*;

public class SwerveModuleSim extends SwerveModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim turnMotor;
    private double angleRotaions = 0;

    public SwerveModuleSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        DCMotor motorsModel = DCMotor.getFalcon500(1);
        driveMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorsModel, DRIVE_MOTOR_MOMENT_OF_INERTIA, DRIVE_MOTOR_GEAR_RATIO),
            motorsModel);
        turnMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorsModel, DRIVE_MOTOR_MOMENT_OF_INERTIA, DRIVE_MOTOR_GEAR_RATIO),
            motorsModel);
    }

    @Override
    protected void periodicBeforeFields() {
        driveMotor.update(0.2);
        turnMotor.update(0.2);

        angleRotaions += (turnMotor.getAngularVelocityRPM() / 60 * 0.02) % 1;
    }

    @Override
    protected double getAbsoluteAngleRotations() {
        return angleRotaions;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setInputVoltage(voltage);
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnMotor.setInputVoltage(voltage);
    }

    @Override
    public void setDrivePercentageSpeed(double speed) {
        driveMotor.setInputVoltage(speed * MAX_VOLTAGE);
    }

    @Override
    public void setTurnPercentageSpeed(double speed) {
        turnMotor.setInputVoltage(speed * MAX_VOLTAGE);
    }    
}
