package frc.robot.subsystems.flywheel.io;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team2679.atlantiskit.logfields.LogFieldsTable;
import frc.robot.subsystems.flywheel.FlyWheelConstants;

public class FlyWheelIOSim extends FlyWheelIO{

    private final FlywheelSim flyWheelMotorSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1),
        FlyWheelConstants.Sim.FLYWHEEL_JKgMetersSquared,FlyWheelConstants.GEAR_RATIO),
         DCMotor.getNeo550(1));

    public FlyWheelIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    @Override
    public double getMotorsRPM(){
        return flyWheelMotorSim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double volt) {
        flyWheelMotorSim.setInputVoltage(volt);
    }
    
}
