package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.hood.io.HoodIO;
import frc.robot.subsystems.hood.io.HoodIOSim;
import frc.robot.subsystems.hood.io.HoodIOSparkMax;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class Hood extends SubsystemBase{

    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final HoodIO io = Robot.isReal()?
        new HoodIOSparkMax(fieldsTable):
        new HoodIOSim(fieldsTable);

    private final RotationalSensorHelper sensorHelpr;

    private final TunableTrapezoidProfile trapezoidProfile = new TunableTrapezoidProfile(
        new TrapezoidProfile.Constraints(
            HoodConstants.MAX_VELOCITY_DEG_PER_SEC,
            HoodConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUEARD)
    );

    private final PIDController hoodPidController = new PIDController(
        HoodConstants.KP,
        HoodConstants.KI,
        HoodConstants.KD
    );
    private final TunableArmFeedforward hoodFeedForward = new TunableArmFeedforward(
        HoodConstants.KS,
        HoodConstants.KG,
        HoodConstants.KV,
        HoodConstants.KA
    );

    public Hood(){
        fieldsTable.update();

        sensorHelpr = new RotationalSensorHelper(io.getHoodMotorAngle(), HoodConstants.ANGLE_OFFSET);    
    }

    public void periodic(){
        sensorHelpr.update(io.getHoodMotorAngle());
    }
    public double calculateFeedForward(double desiredAngle, double desiredSpeed){
        double speed = hoodFeedForward.calculate(Math.toRadians(desiredAngle), desiredSpeed);
        speed += hoodPidController.calculate(getAngleDegrees(), desiredAngle);
        return speed;
    }
    public TrapezoidProfile.State calculateTrapezoidProfile(double time, TrapezoidProfile.State initialState,
    TrapezoidProfile.State desiredState){
        return trapezoidProfile.calculate(time, initialState, desiredState);    
    }
    public void stop(){
        io.setVoltage(0);
    }
    public double getAngleDegrees(){
        return io.getHoodMotorAngle();
    }
    public double getVelocity(){
        return sensorHelpr.getVelocity();
    }
    public void resetPID(){
        hoodPidController.reset();
    }
    public void setVoltage(double volt){
        io.setVoltage(volt);
    }
    
}
