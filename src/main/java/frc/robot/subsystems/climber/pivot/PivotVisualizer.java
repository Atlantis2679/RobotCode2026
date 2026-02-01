package frc.robot.subsystems.climber.pivot;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PivotVisualizer {
    private final LogFieldsTable fieldsTable;
    private String name;

    private final LoggedMechanism2d pivotMech = new LoggedMechanism2d(1.5, 1.5);
    private final LoggedMechanismRoot2d pivotRoot = pivotMech.getRoot("root", 0.75, 0);
    private final LoggedMechanismLigament2d pivotTower;

    public PivotVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color1){
        this.fieldsTable = fieldsTable;
        this.name = name;

        pivotTower = pivotRoot.append(new LoggedMechanismLigament2d("elevator", 0.5, 90, 3, color1));
    }
    public void update(double angle){
        pivotTower.setAngle(angle);
        fieldsTable.recordOutput("Pivot Visualizer Angle", angle);
        fieldsTable.recordOutput(name, pivotMech);
    }
}
