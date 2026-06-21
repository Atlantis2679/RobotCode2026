package frc.robot.subsystems.fourbar;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.MathUtils;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;
import team2679.atlantiskit.valueholders.ValueHolder;

public class FourbarCommands {
    private Fourbar fourbar;

    private SysIdRoutine sysIdRoutine;

    public FourbarCommands(Fourbar fourbar) {
        this.fourbar = fourbar;
        TunablesManager.add("TunableSetVoltages/FourbarSetVoltage", tunableSetVoltage().fullTunable());
        TunablesManager.add("fourbar" + "/TunableMoveToAngle", tunableMoveToAngle().fullTunable());
        TunablesManager.add("fourbar" + "/TunableBounce", tunableBounce().fullTunable());
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null,
                (state) -> Logger.recordOutput("Fourbar SysID State", state.toString())), 
            new SysIdRoutine.Mechanism(
                (volts) -> fourbar.setVoltage(volts.in(Volts), false), 
            null, fourbar));
        TunablesManager.add("fourbar" + "/SysId", sysId());
        
    }

    public Command moveToAngle(DoubleSupplier angle) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return fourbar.runOnce(() -> {
            fourbar.resetPID();
            referenceState.set(new TrapezoidProfile.State(fourbar.getAngleDegrees(), fourbar.getVelocity()));
        }).andThen(fourbar.run(() -> {
            referenceState.set(fourbar.calculateTrapezoidProfile(0.02, referenceState.get(),
                    new TrapezoidProfile.State(angle.getAsDouble(), 0)));
            double voltage = fourbar.calculateFeedForward(referenceState.get().position, referenceState.get().velocity,
                    true);
            fourbar.setVoltage(voltage, true);
        })).finallyDo(fourbar::stop).withName("Move to angle");
    }

    public TunableCommand tunableMoveToAngle() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder angle = tunablesTable.addNumber("angle", 0.0);
            return moveToAngle(angle::get).withName("tunableMoveToAngle");
        });
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command bounce(DoubleSupplier minAngle, DoubleSupplier maxAngle) {
        return moveToAngle(
                () -> MathUtils.cosineWave(minAngle.getAsDouble(), maxAngle.getAsDouble(),
                        Timer.getFPGATimestamp() * 3))
                .withName("bounce");
    }

    public Command bounce(double minAngle, double maxAngle) {
        return bounce(() -> minAngle, () -> maxAngle);
    }

    public TunableCommand tunableBounce() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder changeRate = tunablesTable.addNumber("Change Rate", 1.0);
            DoubleHolder minAngle = tunablesTable.addNumber("minAngle", 0.0);
            DoubleHolder maxAngle = tunablesTable.addNumber("maxAngle", 0.0);
            return moveToAngle(
                    () -> MathUtils.cosineWave(minAngle.get(), maxAngle.get(),
                            Timer.getFPGATimestamp() * changeRate.get()))
                    .withName("tunableBounce");
        });
    }

    public Command sysId() {
        return Commands.sequence(
                this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                this.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                this.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)).withName("sysId");
    }

    public Command manualController(DoubleSupplier speed) {
        return fourbar.run(() -> {
            fourbar.setVoltage(speed.getAsDouble() * FourbarConstants.MAX_VOLTAGE, false);
        }).withName("Fourbar manual controller");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return fourbar.run(() -> fourbar.setVoltage(voltage.get(), true)).finallyDo(fourbar::stop)
                    .withName("Tunable fourbar set voltage");
        });
    }
}
