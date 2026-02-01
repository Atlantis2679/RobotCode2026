package frc.robot.subsystems.climber.pivot.io;

import static frc.robot.subsystems.climber.pivot.PivotConstants.*;
import static frc.robot.subsystems.climber.pivot.PivotConstants.Sim.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PivotIOSim extends PivotIO{
    private final SingleJointedArmSim pivotMotor = new SingleJointedArmSim(
        DCMotor.getNeo550(1),
        GEARING,
        JKG_METERS_SQUEARED,
        ARM_LENGTH_M,
        Math.toRadians(MIN_ANGLE_DEGREES),
        Math.toRadians(MAX_ANGLE_DEGREES),
        true,
        ANGLE_OFFSET    
    );
    public PivotIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }
    
    @Override
    public double getPivotAngleDegrees(){
        return Math.toDegrees(pivotMotor.getAngleRads());
    }

    @Override
    public double getPivotMotorCurrent(){
        return 0;
    }

    @Override
    public boolean getIsEncoderConnected(){
        return true;
    }
    
    @Override
    public void setPivotVoltage(double voltage){
        pivotMotor.setInputVoltage(voltage);
    }
}