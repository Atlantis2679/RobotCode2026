package frc.robot.allCommands;

// import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveCommands;
// import frc.robot.utils.MathUtils.NumberGoalTester;
// import frc.robot.utils.MathUtils.RangeWrapper;
// import team2679.atlantiskit.valueholders.DoubleHolder;
// import team2679.atlantiskit.valueholders.ValueHolder;

public class CommandsWrappers {
    private static final double DRIVE_SPEED = 0.1;
    private static final double ROTATION_SPEED = 0.2;
    private static final double MAX_FLYWHEEL_RPM = 3000;

    final AllCommands allCommands;
    final SwerveCommands swerveCommands;
    private final Trigger trigger;

    double xSpeed = 0;
    double ySpeed = 0;
    double rotationSpeed = 0;

    public CommandsWrappers(AllCommands allCommands, SwerveCommands swerveCommands, Trigger trigger) {
        this.allCommands = allCommands;
        this.swerveCommands = swerveCommands;
        this.trigger = trigger;
        swerveCommands.swerve.setDefaultCommand(driveController());
    }

    private Command driveController() {
        return swerveCommands.driverController(
                () -> ySpeed,
                () -> xSpeed,
                () -> rotationSpeed,
                () -> 0.0,
                () -> false,
                () -> false,
                () -> true
        );
    }

    enum MoveDirection {
        LEFT,
        RIGHT,
        BACK,
        FRONT,
        FRONT_LEFT,
        BACK_LEFT,
        FRONT_RIGHT,
        BACK_RIGHT,
    }

    enum RotateDirection {
        ClockWise,
        CounterClockWise,
    }

    public Command startShooting(double speed, double angle) {
        return new ScheduleCommand(allCommands.getReadyToShoot(() -> speed * MAX_FLYWHEEL_RPM, () -> angle)).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate())).withName("startShooting");
    }

    public Command startLoad() {
        return new ScheduleCommand(allCommands.spinIndex()).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate())).withName("startSpinIndex");
    }

    public Command startIntake() {
        return new ScheduleCommand(allCommands.intake()).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate())).withName("startIntake");
    }

    public Command startMoving(MoveDirection direction) {
        return Commands.runOnce(() -> {
            switch (direction) {
                case LEFT:
                    xSpeed = DRIVE_SPEED;
                    ySpeed = 0;
                    break;
                case RIGHT:
                    xSpeed = -DRIVE_SPEED;
                    ySpeed = 0;
                    break;
                case FRONT:
                    xSpeed = 0;
                    ySpeed = DRIVE_SPEED;
                    break;
                case BACK:
                    xSpeed = 0;
                    ySpeed = -DRIVE_SPEED;
                    break;
                case FRONT_LEFT:
                    xSpeed = DRIVE_SPEED / 2;
                    ySpeed = DRIVE_SPEED / 2;
                    break;
                case BACK_LEFT:
                    xSpeed = DRIVE_SPEED / 2;
                    ySpeed = -DRIVE_SPEED / 2;
                    break;
                case FRONT_RIGHT:
                    xSpeed = -DRIVE_SPEED / 2;
                    ySpeed = DRIVE_SPEED / 2;
                    break;
                case BACK_RIGHT:
                    xSpeed = -DRIVE_SPEED / 2;
                    ySpeed = -DRIVE_SPEED / 2;
                    break;
            }
        }).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command startRotating(RotateDirection direction) {
        return Commands.runOnce(() -> {
            switch (direction) {
                case ClockWise:
                    rotationSpeed = -ROTATION_SPEED;
                    break;
                case CounterClockWise:
                    rotationSpeed = ROTATION_SPEED;
                    break;
            }
        }).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    // public Command rotateBy(RotateDirection direction, double angle) {
    //     if (direction == RotateDirection.CounterClockWise) angle = -angle;
    //     DoubleHolder targetInputAngle = new DoubleHolder(angle);
    //     ValueHolder<NumberGoalTester> goal = new ValueHolder<NumberGoalTester>(null);
    //     return Commands.sequence(
    //         Commands.runOnce(() -> goal.set(new NumberGoalTester(
    //             targetInputAngle.get() + swerveCommands.swerve.getGyroYawDegreesCCW(),
    //             direction == RotateDirection.ClockWise,
    //             Optional.of(new RangeWrapper(0, 360)),
    //             0.08))),
    //         startRotating(direction),
    //         Commands.waitUntil(() -> goal.get().goalReached(swerveCommands.swerve.getGyroYawDegreesCCW())),
    //         stopRotating()
    //     ).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    // }

    public Command stopShooting() {
        return new ScheduleCommand(allCommands.stopFlywheel()).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command stopLoad() {
        return new ScheduleCommand(allCommands.stopIndex()).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command stopIntake() {
        return new ScheduleCommand(allCommands.stopIntake()).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command stopMoving() {
        return Commands.runOnce(() -> {
            xSpeed = 0;
            ySpeed = 0;
        }).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command stopRotating() {
        return Commands.runOnce(() -> rotationSpeed = 0).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    public Command stopAll() {
        return Commands.parallel(
            stopIntake(),
            stopLoad(),
            stopShooting(),
            stopMoving(),
            stopRotating()
        ).andThen(Commands.waitUntil(trigger)).andThen(Commands.waitUntil(trigger.negate()));
    }

    // public Command waitSeconds(double seconds) {
    //     return Commands.waitSeconds(seconds);
    // }

    public Command autoCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> swerveCommands.swerve.resetGyroYawZero()),
            startMoving(MoveDirection.FRONT),
            stopMoving(),
            stopAll()
        );
    }
}
