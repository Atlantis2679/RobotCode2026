package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOFalcon;
import frc.robot.subsystems.swerve.io.SwerveModuleSim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class SwerveModule implements Tunable {
    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;
    private final RotationalSensorHelper angleDegreesCw;
    private final int moduleNum;

    private double lastDriveDistanceMeters;
    private double currentDriveDistanceMeters;

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID,
            int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID)
                : new SwerveModuleSim(fieldsTable);

        fieldsTable.update();

        this.moduleNum = moudleNum;

        angleDegreesCw = new RotationalSensorHelper(io.absoluteTurnAngleRotations.getAsDouble() * 360, OFFSETS[moudleNum]);
        angleDegreesCw.enableContinuousWrap(0, 360);
    }

    public void periodic() {
        angleDegreesCw.update(io.absoluteTurnAngleRotations.getAsDouble() * 360);
        lastDriveDistanceMeters = currentDriveDistanceMeters;
        currentDriveDistanceMeters = getDriveDistanceMeters();

        fieldsTable.recordOutput("Absolute Angle Degrees CW", getDegreesCW());
        fieldsTable.recordOutput("Drive Distance Meters", getDriveDistanceMeters());
        fieldsTable.recordOutput("Module Position", getModulePosition());
        fieldsTable.recordOutput("Module Position Delta", getModulePositionDelta());
    }

    public void setTargetState(SwerveModuleState targetState, boolean optimize, boolean preventJittering,
            boolean useVoltage) {
        if (preventJittering
                && Math.abs(targetState.speedMetersPerSecond) < MAX_SPEED_MPS * PREVENT_JITTERING_MULTIPLAYER) {
            io.setDrivePercentageSpeed(0);
            return;
        }

        if (optimize)
            targetState.optimize(new Rotation2d(Math.toRadians(getDegreesCW())));

        if (useVoltage)
            io.setDriveVoltage((targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
        else
            io.setDrivePercentageSpeed(targetState.speedMetersPerSecond / MAX_SPEED_MPS);

        io.setTurnAngleRotations(targetState.angle.getRotations());
    }

    public double getDegreesCW() {
        return angleDegreesCw.getAngle();
    }

    public int getModuleNumber() {
        return moduleNum;
    }

    public double getDriveDistanceMeters() {
        return io.driveDistanceRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), Rotation2d.fromDegrees(getDegreesCW()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(getDriveDistanceMeters() - lastDriveDistanceMeters,
                Rotation2d.fromDegrees(getDegreesCW()));
    }

    @Override
    public void initTunable(TunableBuilder builder) {
    }
}
