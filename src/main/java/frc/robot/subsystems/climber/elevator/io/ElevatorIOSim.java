package frc.robot.subsystems.climber.elevator.io;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.climber.elevator.ElevatorConstants.Sim.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ElevatorIOSim extends ElevatorIO {
    private final ElevatorSim elevatorMotor = new ElevatorSim(
        DCMotor.getNEO(1), 
        GEARING, 
        CARRIGE_MASS_KG, 
        DRUM_RADIUS, 
        MIN_HEIGHT_METERS, 
        MAX_HEIGHT_METERS, 
        true, 
        0, 
        0.01);

    public ElevatorIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public double getElevatorHeight() {
        return elevatorMotor.getPositionMeters();
    }

    public double getElevatorMotorCurrent() {
        return 0;
    }

    protected boolean getIsEncoderConnected() {
        return true;
    }

    public void setElevatorVoltage(double voltage) {
        elevatorMotor.setInputVoltage(voltage);
    }
}
