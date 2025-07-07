package frc.robot.subsystems;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.controller.PIDController;
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
    private final RotationalSensorHelper angleDegreesCW;
    private final PIDController drivePIDController;
    private final PIDController turnPIDController;

    public SwerveModule(LogFieldsTable swerveFieldsTable, int moudleNum, int driveMotorID, int turnMotorID, int canCoderID) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moudleNum + " " + getModuleName(moudleNum));
        io = Robot.isReal() ? 
            new SwerveModuleIOFalcon(fieldsTable, moudleNum, driveMotorID, turnMotorID, canCoderID) : 
            new SwerveModuleSim(fieldsTable);
    
        fieldsTable.update();
        
        angleDegreesCW = new RotationalSensorHelper(io.angleRotations.getAsDouble() * 360);
        angleDegreesCW.enableContinuousWrap(0, 360);

        drivePIDController = new PIDController(DRIVE_MOTOR_KP, DRIVE_MOTOR_KI, DRIVE_MOTOR_KD);
        turnPIDController = new PIDController(TURN_MOTOR_KP, TURN_MOTOR_KI, TURN_MOTOR_KD);
    }

    public void periodic() {
        angleDegreesCW.update(io.angleRotations.getAsDouble() * 360);
        fieldsTable.recordOutput("angleDegreesCW", angleDegreesCW.getAngle());
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("driveMotorPID", drivePIDController);
        builder.addChild("turnMotorPID", turnPIDController);
    }
}
