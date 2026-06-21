package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandsUtils {
    public static Command dynamicSwitchBetweenCommands(BooleanSupplier condition, Command onTrue, Command onFalse) {
        return dynamicSwitchBetweenCommands(condition, () -> !condition.getAsBoolean(), onTrue, onFalse);
    }

    public static Command dynamicSwitchBetweenCommands(BooleanSupplier switchToFirst,
            BooleanSupplier switchToSecond,
            Command first, Command second) {
        return Commands.repeatingSequence(
            first.until(switchToSecond),
            second.until(switchToFirst));
    }
}
