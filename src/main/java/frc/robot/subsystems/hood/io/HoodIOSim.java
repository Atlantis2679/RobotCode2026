package frc.robot.subsystems.hood.io;

import static frc.robot.subsystems.climber.pivot.PivotConstants.MAX_ANGLE_DEGREES;
import static frc.robot.subsystems.climber.pivot.PivotConstants.MIN_ANGLE_DEGREES;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;
import frc.robot.subsystems.hood.*;
import frc.robot.subsystems.hood.HoodConstants.Sim;

public class HoodIOSim extends HoodIO {
    
    private final SingleJointedArmSim hoodMotor = new SingleJointedArmSim(
        DCMotor.getNeo550(1),
        Sim.JOINT_GEAR_RATIO,
        Sim.JKG_METERS_SQUEARED,
        Sim.ARM_LENGTH_M,
        Math.toRadians(MIN_ANGLE_DEGREES),
        Math.toRadians(MAX_ANGLE_DEGREES),
        true,
        HoodConstants.ANGLE_OFFSET    
    );

    public HoodIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    public void periodicBeforeFields(){
        hoodMotor.update(0.02);
    }

    @Override
    public double getHoodMotorAngleDegree() {
        return Math.toDegrees(hoodMotor.getAngleRads());
    }

    @Override
    public void setVoltage(double volt) {
        hoodMotor.setInputVoltage(volt);
    }
  
    @Override
    protected boolean getIsEncoderConnected() {
        return false;
    }
}
