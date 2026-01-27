package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class ClimberVisualizer {
    private final LogFieldsTable fieldsTable;
    private String name;

    private final LoggedMechanism2d elevatorMech = new LoggedMechanism2d(1.5, 1.5);
    private final LoggedMechanismRoot2d elevatorRoot = elevatorMech.getRoot("root", 0.75, 0);
    private final LoggedMechanismLigament2d elevatorTower;

    private final LoggedMechanismLigament2d pivot;

    public ClimberVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color1, Color8Bit color2){
        this.fieldsTable = fieldsTable;
        this.name = name;

        elevatorTower = elevatorRoot.append(new LoggedMechanismLigament2d("elevator", 0.5, 90, 3, color1));
        pivot = elevatorTower.append(new LoggedMechanismLigament2d("pivot", 0.2, 0, 0.5, color2));
    }
    public void update(double height, double angle){
        elevatorTower.setLength(height);
        pivot.setAngle(angle);
        fieldsTable.recordOutput("Visualizer Elevator Height", height);
        fieldsTable.recordOutput("Visualizer Pivot Angle", angle);
        fieldsTable.recordOutput(name, elevatorMech);
    }
}
