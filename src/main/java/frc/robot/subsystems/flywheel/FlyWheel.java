package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.io.*;
import static frc.robot.subsystems.flywheel.FlyWheelConstants.*;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableSimpleMotorFeedforward;

public class FlyWheel extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final FlyWheelIO io = Robot.isReal() 
        ? new FlyWheelIOFalcon(fieldsTable) 
        : new FlyWheelIOSim(fieldsTable);

    private PIDController flyWheelPidController = new PIDController(KP, KI, KD);
    
    private TunableSimpleMotorFeedforward flyWheelFeedforward = Robot.isSimulation() ?
        new TunableSimpleMotorFeedforward(Sim.SIM_KS, Sim.SIM_KV, Sim.SIM_KA) :
        new TunableSimpleMotorFeedforward(KS, KV, KA);

    public FlyWheel() {
        fieldsTable.update();
        TunablesManager.add(getName(), (Tunable) this);
    }

    @Override
    public void periodic(){
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        fieldsTable.recordOutput("Current diff", Math.abs(io.motor1Current.getAsDouble() - io.motor2Current.getAsDouble()));
        SmartDashboard.putNumber("Motors RPM", getMotorsRPM());
    }

    public double getMotorsRPM(){
        return io.motorsRPM.getAsDouble();
    }

    public void setVoltage(double volt){
        io.setVoltage(MathUtil.clamp(volt, -MAX_VOLTAGE, MAX_VOLTAGE));
    }

    public double calculateFeedForward(double desiredSpeed, boolean usePID) {
        double speed = flyWheelFeedforward.calculate(desiredSpeed);
        if (usePID) {
            speed += flyWheelPidController.calculate(getMotorsRPM(), desiredSpeed);
        }
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

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("PID Controller", flyWheelPidController);
        builder.addChild("FeedForward", flyWheelFeedforward);
    }
}
