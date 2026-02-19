package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodCommands {
    private final Hood hood;

    public HoodCommands(Hood hood) {
        this.hood = hood;
        TunablesManager.add("TunableSetVoltages/HoodSetVoltage", tunableSetVoltage().fullTunable());
        TunablesManager.add("Tunable hood cosine follower", hoodCosineWaveFollower().fullTunable());
    }

    public Command moveToAngle(DoubleSupplier angle) {
        return hood.runOnce(() -> {
            hood.resetPID();
        }).andThen(hood.run(() -> {
            hood.setVoltage(hood.calculatePID(angle.getAsDouble()));
        })).withName("Hood move to angle");
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return hood.run(() -> hood.setVoltage(voltage.get())).finallyDo(hood::stop)
                    .withName("Tunable hood set voltage");
        });
    }

    public Command hoodDemo() {
        DoubleHolder counter = new DoubleHolder(0);
        DoubleHolder setpoint = new DoubleHolder(0);
        return hood.runOnce(() -> {counter.set(0);}).andThen(() -> {
            if (counter.get() >= 100) {
                counter.set(0);
            }
            if (counter.get() == 0) setpoint.set(Math.random() * 40);
            counter.set(counter.get() + 1);
            hood.setVoltage(hood.calculatePID(setpoint.get()));
        });
    }

    public TunableCommand hoodCosineWaveFollower() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder changeRate = tunablesTable.addNumber("Change Rate (Scalar)", 1.0);
            return hood.run(() -> {
                double angle = cosineWaveFollower(hood.minAngle, hood.maxAngle, Timer.getFPGATimestamp() * changeRate.get());
                double voltage = hood.calculatePID(angle);
                hood.setVoltage(voltage);
            });
        });
    }

    public Command manualController(DoubleSupplier speed) {
        return hood.run(() -> {
            hood.setVoltage(speed.getAsDouble() * MAX_VOLTAGE);
        }).finallyDo(hood::stop).withName("Hood manual controller");
    }

    public static double cosineWaveFollower(double a, double b, double x) {
        double average = (a + b) / 2;
        double delta = (a - b) / 2; 
        return average + delta * Math.cos(x);
    }
}
