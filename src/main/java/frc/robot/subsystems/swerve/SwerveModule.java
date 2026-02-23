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
    private final RotationalSensorHelper absoluteAngleDegreesCWW;
    private final int moduleNum;

    private double lastDriveDistanceMeters;
    private double currentDriveDistanceMeters;

    private double currentAngleDegreesCCW;

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID,
            int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID)
                : new SwerveModuleSim(fieldsTable);

        fieldsTable.update();

        this.moduleNum = moudleNum;

        absoluteAngleDegreesCWW = new RotationalSensorHelper(io.absoluteTurnAngleRotations.getAsDouble() * 360, OFFSETS[moudleNum]);
        absoluteAngleDegreesCWW.enableContinuousWrap(0, 360);

        resetIntegratedAngleToAbsolute();
    }

    public void periodic() {
        absoluteAngleDegreesCWW.update(io.absoluteTurnAngleRotations.getAsDouble() * 360);
        lastDriveDistanceMeters = currentDriveDistanceMeters;
        currentDriveDistanceMeters = getDriveDistanceMeters();

        currentAngleDegreesCCW = getIntegratedDegreesCCW();

        fieldsTable.recordOutput("Absolute Angle Degrees CCW", getAbsoluteDegreesCCW());
        fieldsTable.recordOutput("Integrated Angles Degrees CCW", getIntegratedDegreesCCW());
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
            targetState.optimize(Rotation2d.fromDegrees(currentAngleDegreesCCW));

        fieldsTable.recordOutput("Module target state", targetState);
        if (useVoltage) {
            fieldsTable.recordOutput("Drive motor target voltage", (targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
            io.setDriveVoltage((targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
        } else {
            fieldsTable.recordOutput("Drive motor target speed", targetState.speedMetersPerSecond / MAX_SPEED_MPS);
            io.setDrivePercentageSpeed(targetState.speedMetersPerSecond / MAX_SPEED_MPS);
        }

        fieldsTable.recordOutput("Turn motor target rotation", targetState.angle.getRotations());
        io.setTurnAngleRotations(targetState.angle.getRotations());
    }

    public double getAbsoluteDegreesCCW() {
        return absoluteAngleDegreesCWW.getAngle();
    }

    public int getModuleNumber() {
        return moduleNum;
    }

    public double getDriveDistanceMeters() {
        return io.driveDistanceRotations.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), Rotation2d.fromDegrees(getIntegratedDegreesCCW()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(getDriveDistanceMeters() - lastDriveDistanceMeters,
                Rotation2d.fromDegrees(getIntegratedDegreesCCW()));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMPS(), getRotation2d());
    }
    
    public double getVelocityMPS() {
        return io.driveSpeedRPS.getAsDouble() * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAbsoluteDegreesCCW());
    }

    public double getIntegratedDegreesCCW() {
        return io.intergatedTurnAngleRotations.getAsDouble() * 360;
    }

    public void resetIntegratedAngleToAbsolute() {
        currentAngleDegreesCCW = getAbsoluteDegreesCCW();
        io.resetIntegratedAngleRotations(currentAngleDegreesCCW / 360);
    }

    public void resetAngleDegreesCCW(double newAngle) {
        absoluteAngleDegreesCWW.resetAngle(newAngle);
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
        builder.addDoubleProperty("Integrated Angle Degrees CCW", this::getIntegratedDegreesCCW, null);
        builder.addDoubleProperty("Absolute Angle Degrees CCW", this::getAbsoluteDegreesCCW, null);
        builder.addDoubleProperty("Tunable Offset", absoluteAngleDegreesCWW::getOffset, newOffset -> {
            absoluteAngleDegreesCWW.setOffset(newOffset);
            resetIntegratedAngleToAbsolute();
        });
    }
}
