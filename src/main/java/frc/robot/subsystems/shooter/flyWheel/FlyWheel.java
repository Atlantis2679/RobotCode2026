package frc.robot.subsystems.shooter.flyWheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.flyWheel.io.*;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableArmFeedforward;
import team2679.atlantiskit.tunables.extensions.TunableTrapezoidProfile;

public class FlyWheel extends SubsystemBase{
    
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final FlyWheelIO io = Robot.isReal()?new FlyWheelIOSparkMax(fieldsTable) :
        new FlyWheelIOSim(fieldsTable);

    private final RotationalSensorHelper sensorHelper;

    private PIDController flyWheelPidController = new PIDController(
        FlyWheelConstants.KP,
        FlyWheelConstants.KI,
        FlyWheelConstants.KD
    );
    private final TunableTrapezoidProfile flyWheelTrapezoid = new TunableTrapezoidProfile(
        new TrapezoidProfile.Constraints(
            FlyWheelConstants.MAX_VELOCITY_DEG_PER_SEC,
            FlyWheelConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUEARD)        
    );
    private TunableArmFeedforward flyWheelFeedforward = Robot.isSimulation()?
        new TunableArmFeedforward(
            FlyWheelConstants.Sim.SIM_KS,
            FlyWheelConstants.Sim.SIM_KG,
            FlyWheelConstants.Sim.SIM_KV,
            FlyWheelConstants.Sim.SIM_KA
            ):
        new TunableArmFeedforward(
            FlyWheelConstants.KS,
            FlyWheelConstants.KG,
            FlyWheelConstants.KV,
            FlyWheelConstants.KA
    );


    public FlyWheel() {
        fieldsTable.update();

        sensorHelper = new RotationalSensorHelper(io.flywheelMotorAbsoluteRotations.getAsDouble(), 0);
    }

    @Override
    public void periodic(){
        sensorHelper.update(io.flywheelMotorAbsoluteRotations.getAsDouble());

        fieldsTable.recordOutput("current command", getCurrentCommand() == null?
        getCurrentCommand().getName() : "None");

        SmartDashboard.putNumber("FlyWheel abs rotations", getAbsoluteRotations());
    }


    public double getAbsoluteRotations(){
        return io.getAbsoluteRotations();
    }
    public double getVelocity(){
        return sensorHelper.getVelocity();
    }
    public void setVoltage(double volt){
        io.setVoltage(volt);
    }
    public void setRPM(double desireRPM){

    }
    public TrapezoidProfile.State calculateTrapezoidProfile(
        double time, TrapezoidProfile.State initialState, TrapezoidProfile.State desiredState){
            return flyWheelTrapezoid.calculate(time, initialState, desiredState);
    }
    public double CalcVolts(double desiredSpeed, double desiredExceleration){
        double exceleration = flyWheelFeedforward.calculate(desiredSpeed, desiredExceleration);
        exceleration += flyWheelPidController.calculate(getVelocity(), desiredSpeed);
        return exceleration;
    }

    public void stop(){
        io.setVoltage(0);
    }
    public void resetPID(){
        flyWheelPidController.reset();
    }
}
