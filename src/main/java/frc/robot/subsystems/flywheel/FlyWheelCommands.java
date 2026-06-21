package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class FlyWheelCommands {
    private final FlyWheel flyWheel;

    public final SysIdRoutine sysIdRoutine;

    public FlyWheelCommands(FlyWheel flyWheel) {
        this.flyWheel = flyWheel;
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> flyWheel.setVoltage(volts.in(Volts)), null, flyWheel));
        TunablesManager.add("TunableSetVoltages/FlywheelSetVoltage", tunableSetVoltage().fullTunable());
        TunablesManager.add(flyWheel.getName() + "/SysId", executeSysID());
    }

    public Command reachSpeed(DoubleSupplier speedRPM) {
        return flyWheel.runOnce(() -> flyWheel.resetPID()).andThen(flyWheel.run(() -> {
            flyWheel.setVoltage(flyWheel.calculateVoltage(speedRPM.getAsDouble(), true));
        })).finallyDo(flyWheel::stop).withName("Flywheel reach speed");
    }

    public Command manualController(DoubleSupplier precentageVoltage) {
        return flyWheel.run(() -> {
            flyWheel.setVoltage(precentageVoltage.getAsDouble() * FlyWheelConstants.MAX_VOLTAGE);
        }).finallyDo(flyWheel::stop).withName("Flywheel manual controller");
    }

    public Command executeSysID() {
        return Commands.sequence(
            Commands.runOnce(() -> SignalLogger.start()),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
            sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.runOnce(() -> SignalLogger.stop())
        ).withName("sysID");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return flyWheel.run(() -> flyWheel.setVoltage(voltage.get())).finallyDo(flyWheel::stop)
                    .withName("Tunable Flywheel set voltage");
        });
    }
}
