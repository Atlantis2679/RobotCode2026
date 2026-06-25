package frc.robot.subsystems.fourbar;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.MathUtils;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.BooleanHolder;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class FourbarCommands {
    private Fourbar fourbar;

    public FourbarCommands(Fourbar fourbar) {
        this.fourbar = fourbar;
        TunablesManager.add("TunableSetVoltages/FourbarSetVoltage", tunableSetVoltage().fullTunable());
        TunablesManager.add(fourbar.getName() + "/TunableBounce", tunableBounce().fullTunable());        
    }

    public Command runWithVoltage(DoubleSupplier voltage) {
        BooleanHolder lastSign = new BooleanHolder(voltage.getAsDouble() >= 0);
        return Commands.repeatingSequence(
            fourbar.run(() -> fourbar.setVoltage(voltage.getAsDouble())).until(fourbar::isStuck),
            fourbar.run(fourbar::stop).until(() -> (voltage.getAsDouble() >= 0) != lastSign.get()),
            Commands.runOnce(() -> lastSign.set(voltage.getAsDouble() >= 0)))
            .finallyDo(fourbar::stop).withName("runWithVoltage");
    }

    public Command runWithVoltage(double voltage) {
        return runWithVoltage(() -> voltage);
    }

    public Command bounce(MathUtils.CosineWaveFollower cosineWaveFollower) {
        return runWithVoltage(cosineWaveFollower::getNext).withName("bounce");
    }

    public TunableCommand tunableBounce() {
        return TunableCommand.wrap((tunablesTable) -> {
            MathUtils.CosineWaveFollower cosineWaveFollower = new MathUtils.CosineWaveFollower(0, 0);
            tunablesTable.addChild("Cosine wave follower", cosineWaveFollower);
            return bounce(cosineWaveFollower).withName("tunableBounce");
        });
    }

    public Command manualController(DoubleSupplier speed) {
        return fourbar.run(() -> {
            fourbar.setVoltage(speed.getAsDouble() * FourbarConstants.MAX_VOLTAGE);
        }).withName("Fourbar manual controller");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return runWithVoltage(voltage::get).withName("Tunable fourbar set voltage");
        });
    }
}
