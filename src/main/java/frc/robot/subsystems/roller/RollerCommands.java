package frc.robot.subsystems.roller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class RollerCommands {
    private Roller roller;

    public RollerCommands(Roller roller) {
        this.roller = roller;
        TunablesManager.add("TunableSetVoltages/RollerSetVoltage", tunableSetVoltage().fullTunable());
    }

    public Command spin(DoubleSupplier voltage) {
        return roller.run(() -> roller.setVoltage(voltage.getAsDouble()))
                .finallyDo(roller::stop)
                .withName("Roller Spin");
    }

    public Command spin(double speed) {
        return spin(() -> speed);
    }

    public Command manualController(DoubleSupplier speed) {
        return roller.run(() -> roller.setVoltage(speed.getAsDouble() * RollerConstants.MAX_VOLTAGE))
                .finallyDo(roller::stop)
                .withName("Roller manual controller");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return spin(voltage::get)
                .withName("Tunable Flywheel set voltage");
        });
    }
}
