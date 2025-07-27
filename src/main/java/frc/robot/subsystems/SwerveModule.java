package frc.robot.subsystems;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

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
    private final RotationalSensorHelper absoluteAngleDegreesCw;

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID, int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? 
            new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID) : 
            new SwerveModuleSim(fieldsTable);
    
        fieldsTable.update();
        
        absoluteAngleDegreesCw = new RotationalSensorHelper(io.absoluteAngleRotations.getAsDouble() * 360);
        absoluteAngleDegreesCw.enableContinuousWrap(0, 360);
    }

    public void periodic() {
        absoluteAngleDegreesCw.update(io.absoluteAngleRotations.getAsDouble() * 360);
        fieldsTable.recordOutput("absoluteAngleDegreesCW", absoluteAngleDegreesCw.getAngle());
    }

    @Override
    public void initTunable(TunableBuilder builder) {
    }
}
