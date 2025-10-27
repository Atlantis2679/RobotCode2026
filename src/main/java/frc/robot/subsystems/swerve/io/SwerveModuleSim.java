package frc.robot.subsystems.swerve.io;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;
import static frc.robot.subsystems.swerve.SwerveConstants.Sim.*;

public class SwerveModuleSim extends SwerveModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim turnMotor;
    private double angleRotaions = 0;
    private double driveMotorRotations = 0;

    private final PIDController turnPIDController = new PIDController(SIM_TURN_MOTOR_KP, SIM_TURN_MOTOR_KI,
            SIM_TURN_MOTOR_KD);

    public SwerveModuleSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        DCMotor motorsModel = DCMotor.getFalcon500(1);
        driveMotor = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motorsModel, DRIVE_MOTOR_MOMENT_OF_INERTIA, DRIVE_GEAR_RATIO),
                motorsModel);
        turnMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorsModel, TURN_GEAR_RATIO, TURN_GEAR_RATIO),
                motorsModel);
    }

    @Override
    protected void periodicBeforeFields() {
        driveMotor.update(0.2);
        turnMotor.update(0.2);

        angleRotaions += (turnMotor.getAngularVelocityRPM() / 60 * 0.02);
        angleRotaions = warpAngle(angleRotaions);
    }

    private double warpAngle(double angle) {
        return ((angle + 1) % 2 + 2) % 2 - 1;
    }

    @Override
    protected double getAbsoluteTurnAngleRotations() {
        return angleRotaions;
    }

    @Override
    protected double getIntegratedTurnAngleRotations() {
        return angleRotaions;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setInputVoltage(voltage);
    }

    @Override
    public void setDrivePercentageSpeed(double speed) {
        driveMotor.setInputVoltage(speed * MAX_VOLTAGE);
    }

    @Override
    public void setTurnAngleRotations(double rotations) {
        turnMotor.setInputVoltage(rotations);
    }

    @Override
    protected double getDriveDistanceRotations() {
        return driveMotorRotations;
    }

    @Override
    public void setCoast() {
    }
}
