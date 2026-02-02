package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.io.*;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.extensions.TunableSimpleMotorFeedforward;

public class FlyWheel extends SubsystemBase{
    
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final FlyWheelIO io = Robot.isReal()?new FlyWheelIOSparkMax(fieldsTable) :
        new FlyWheelIOSim(fieldsTable);


    private PIDController flyWheelPidController = new PIDController(
        FlyWheelConstants.KP,
        FlyWheelConstants.KI,
        FlyWheelConstants.KD
    );
    private TunableSimpleMotorFeedforward flyWheelFeedforward = Robot.isSimulation()?
        new TunableSimpleMotorFeedforward(
            FlyWheelConstants.Sim.SIM_KS,
            FlyWheelConstants.Sim.SIM_KV,
            FlyWheelConstants.Sim.SIM_KA
            ):
        new TunableSimpleMotorFeedforward(
            FlyWheelConstants.KS,
            FlyWheelConstants.KV,
            FlyWheelConstants.KA
    );


    public FlyWheel() {
        fieldsTable.update();

    }

    @Override
    public void periodic(){

        fieldsTable.recordOutput("current command", getCurrentCommand() == null?
        getCurrentCommand().getName() : "None");

        SmartDashboard.putNumber("Motors RPM",getMotorsRPM());
    }


    public double getMotorsRPM(){
        return io.flywheelmotorsRPM.getAsDouble();
    }
    public void setVoltage(double volt){
        io.setVoltage(MathUtil.clamp(volt, -FlyWheelConstants.MAX_VOLTAGE, FlyWheelConstants.MAX_VOLTAGE));
    }

    public double calcVoltsforRPM(double desiredSpeed){
        double speed = flyWheelFeedforward.calculate(desiredSpeed);
        speed += flyWheelPidController.calculate(getMotorsRPM(), desiredSpeed);
        return speed;
    }

    public boolean isAtSpeed(double targetSpeedRpm){
        boolean isAtSpeed = Math.abs(targetSpeedRpm - getMotorsRPM()) < FlyWheelConstants.SPEED_TOLERANCE_RPM;
        SmartDashboard.putBoolean("Flywheel at speed:", isAtSpeed);
        fieldsTable.recordOutput("Flywheel at speed", isAtSpeed);
        return isAtSpeed;
    }

    public void stop(){
        io.setVoltage(0);
    }
    public void resetPID(){
        flyWheelPidController.reset();
    }
}
