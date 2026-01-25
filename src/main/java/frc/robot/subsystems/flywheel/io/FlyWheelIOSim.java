package frc.robot.subsystems.flywheel.io;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import team2679.atlantiskit.logfields.LogFieldsTable;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.FlyWheelConstants.Sim;

public class FlyWheelIOSim extends FlyWheelIO{

    public FlyWheelIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
        //FlywheelSim sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1),FlyWheelConstants.Sim.FLYWHEEL_JKgMetersSquared,FlyWheelConstants.GEAR_RATIO), 2, );
    }

    @Override
    public double getMotorsRPM(){
        return 0;
    }

    @Override
    public void setVoltage(double volt) {
    }
    
}
