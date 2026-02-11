package frc.robot.subsystems.hood.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.hood.HoodConstants.ANGLE_TOLERENCE_DEGREES;
import static frc.robot.subsystems.hood.HoodConstants.MAX_ANGLE_DEGREES;
import static frc.robot.subsystems.hood.HoodConstants.MIN_ANGLE_DEGREES;
import static frc.robot.subsystems.hood.HoodConstants.Sim.*;

public class HoodIOSim extends HoodIO {
    private final SingleJointedArmSim motor = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            JOINT_GEAR_RATIO,
            JKG_METERS_SQUEARED,
            ARM_LENGTH_M,
            Math.toRadians(MIN_ANGLE_DEGREES),
            Math.toRadians(MAX_ANGLE_DEGREES),
            true,
            0);

    public HoodIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public void periodicBeforeFields() {
        motor.update(0.02);
    }

    @Override
    public double getMotorRotations() {
        return motor.getAngleRads() / (Math.PI * 2);
    }

    @Override
    public void setVoltage(double volt) {
        motor.setInputVoltage(volt);
    }

    @Override
    public void setCoast() {
    }

    @Override
    protected boolean limitSwitch() {
        return Math.abs(Math.toDegrees(motor.getAngleRads())) <= ANGLE_TOLERENCE_DEGREES;
    }

    @Override
    protected double getMotorCurrent() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    public void resetRotation() {
        motor.setState(0, motor.getVelocityRadPerSec());
    }

}
