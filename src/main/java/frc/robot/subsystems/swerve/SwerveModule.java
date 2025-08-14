package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID, int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? 
            new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID) : 
            new SwerveModuleSim(fieldsTable);
    
        fieldsTable.update();

        this.moduleNum = moudleNum;
        
        angleDegreesCw = new RotationalSensorHelper(io.absoluteAngleRotations.getAsDouble() * 360);
        angleDegreesCw.enableContinuousWrap(-180, 180);
    }

    public void periodic() {
        angleDegreesCw.update(io.absoluteAngleRotations.getAsDouble() * 360);
        fieldsTable.recordOutput("Absolute Angle Degrees CW", getDegreesCW());
    }

    public void setTargetState(SwerveModuleState targetState, boolean optimize, boolean useVoltage) {
        if (optimize)
            targetState.optimize(new Rotation2d(Math.toRadians(getDegreesCW())));
    
        if (useVoltage)
            io.setDriveVoltage((targetState.speedMetersPerSecond / MAX_SPEED_MPS) * MAX_VOLTAGE);
        else
            io.setDrivePercentageSpeed(targetState.speedMetersPerSecond / MAX_SPEED_MPS);
        
        io.set(targetState.angle.getRotations());
    }

    public double getDegreesCW() {
        return io.absoluteAngleRotations.getAsDouble();
    }

    public int getModuleNumber() {
        return moduleNum;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
    }
}
