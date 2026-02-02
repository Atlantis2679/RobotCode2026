package frc.robot.subsystems.climber.pivot.io;

import static frc.robot.subsystems.climber.pivot.PivotConstants.*;
import static frc.robot.subsystems.climber.pivot.PivotConstants.Sim.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO {
    private final SingleJointedArmSim motor = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            GEARING,
            JKG_METERS_SQUEARED,
            ARM_LENGTH_M,
            Math.toRadians(MIN_ANGLE_DEGREES),
            Math.toRadians(MAX_ANGLE_DEGREES),
            true,
            ANGLE_OFFSET);

    public PivotIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        motor.update(0.02);
    }

    @Override
    public double getAngleDegrees() {
        return Math.toDegrees(motor.getAngleRads());
    }

    @Override
    public double getPivotMotorCurrent() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    public boolean getIsEncoderConnected() {
        return true;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }
}