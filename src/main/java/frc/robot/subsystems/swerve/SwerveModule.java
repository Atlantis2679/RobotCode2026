package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import edu.wpi.first.math.controller.PIDController;
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
    private final RotationalSensorHelper absoluteAngleDegreesCW;
    private final int moduleNum;

    private double lastDriveDistanceMeters;
    private double currentDriveDistanceMeters;

    private double currentAngleDegreesCW;

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID,
            int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID)
                : new SwerveModuleSim(fieldsTable);

        fieldsTable.update();

        this.moduleNum = moudleNum;

        absoluteAngleDegreesCW = new RotationalSensorHelper(io.absoluteTurnAngleRotations.getAsDouble() * 360, OFFSETS[moudleNum]);
        absoluteAngleDegreesCW.enableContinuousWrap(0, 360);

        resetIntegratedAngleToAbsolute();
    }

    public void periodic() {
        absoluteAngleDegreesCW.update(io.absoluteTurnAngleRotations.getAsDouble() * 360);
        lastDriveDistanceMeters = currentDriveDistanceMeters;
        currentDriveDistanceMeters = getDriveDistanceMeters();

        currentAngleDegreesCW = getIntegratedDegreesCW();

        fieldsTable.recordOutput("Absolute Angle Degrees CW", getAbsoluteDegreesCW());
        fieldsTable.recordOutput("Integrated Angles Degrees CW", getIntegratedDegreesCW());
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
            targetState.optimize(Rotation2d.fromDegrees(-currentAngleDegreesCW));

        if (useVoltage) {
            fieldsTable.recordOutput("Drive motor desired voltage", (targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
            io.setDriveVoltage((targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
        } else {
            fieldsTable.recordOutput("Drive motor desired speed", targetState.speedMetersPerSecond / MAX_SPEED_MPS);
            io.setDrivePercentageSpeed(targetState.speedMetersPerSecond / MAX_SPEED_MPS);
        }

        fieldsTable.recordOutput("Turn motor desired rotation", targetState.angle.getRotations());
        io.setTurnAngleRotations(targetState.angle.getRotations());
    }

    public double getAbsoluteDegreesCW() {
        return absoluteAngleDegreesCW.getAngle();
    }

    public int getModuleNumber() {
        return moduleNum;
    }

    public double getDriveDistanceMeters() {
        return io.driveDistanceRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), Rotation2d.fromDegrees(-getIntegratedDegreesCW()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(getDriveDistanceMeters() - lastDriveDistanceMeters,
                Rotation2d.fromDegrees(-getIntegratedDegreesCW()));
    }

    public double getIntegratedDegreesCW() {
        return io.intergatedTurnAngleRotations.getAsDouble() * 360;
    }

    public void resetIntegratedAngleToAbsolute() {
        currentAngleDegreesCW = getAbsoluteDegreesCW();
        io.resetIntegratedAngleRotations(currentAngleDegreesCW / 360);
    }

    public void resetAngleDegrees(double newAngle) {
        absoluteAngleDegreesCW.resetAngle(newAngle);
        resetIntegratedAngleToAbsolute();
    }

    public void setCoast() {
        io.setCoast();
    }

    public void setTurnPID(PIDController pidController) {
        io.setTurnKP(pidController.getP());
        io.setTurnKI(pidController.getI());
        io.setTurnKD(pidController.getD());
    }

    public PIDController getTurnPID() {
        return new PIDController(io.turnKP.getAsDouble(), io.turnKI.getAsDouble(), io.turnKD.getAsDouble());
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addDoubleProperty("Integrated Angle Degrees CW", this::getIntegratedDegreesCW, null);
        builder.addDoubleProperty("Absolute Angle Degrees CW", this::getAbsoluteDegreesCW, null);
        builder.addDoubleProperty("Tunable Offset", absoluteAngleDegreesCW::getOffset, newOffset -> {
            absoluteAngleDegreesCW.setOffset(newOffset);
            resetIntegratedAngleToAbsolute();
        });
    }
}
