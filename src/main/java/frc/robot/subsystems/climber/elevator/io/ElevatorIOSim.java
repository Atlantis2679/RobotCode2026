package frc.robot.subsystems.climber.elevator.io;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.DRUM_RADIUS_METERS;
import static frc.robot.subsystems.climber.elevator.ElevatorConstants.MAX_HEIGHT_METERS;
import static frc.robot.subsystems.climber.elevator.ElevatorConstants.MIN_HEIGHT_METERS;
import static frc.robot.subsystems.climber.elevator.ElevatorConstants.Sim.CARRIGE_MASS_KG;
import static frc.robot.subsystems.climber.elevator.ElevatorConstants.Sim.GEARING;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSim extends ElevatorIO {
    private final ElevatorSim motor = new ElevatorSim(
        DCMotor.getNEO(1), 
        GEARING, 
        CARRIGE_MASS_KG, 
        DRUM_RADIUS_METERS, 
        MIN_HEIGHT_METERS, 
        MAX_HEIGHT_METERS, 
        true, 
        0, 
        0.01);

    public ElevatorIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected void periodicBeforeFields() {
        motor.update(0.02);
    }

    @Override
    public double getHeightMeters() {
        return motor.getPositionMeters();
    }

    @Override
    public double getMotorCurrent() {
        return motor.getCurrentDrawAmps();
    }

    @Override
    protected boolean getIsEncoderConnected() {
        return true;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setInputVoltage(voltage);
    }
}
