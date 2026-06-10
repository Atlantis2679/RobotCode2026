package frc.robot.allCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveCommands;

public class CommandsWrappers {
    private static final double DRIVE_SPEED = 0.05;
    private static final double ROTATION_SPEED = 0.05;
    private static final double MAX_FLYWHEEL_RPM = 3000;

    final AllCommands allCommands;
    final SwerveCommands swerveCommands;
    double xSpeed = 0;
    double ySpeed = 0;
    double rotationSpeed = 0;

    public CommandsWrappers(AllCommands allCommands, SwerveCommands swerveCommands) {
        this.allCommands = allCommands;
        this.swerveCommands = swerveCommands;
        swerveCommands.swerve.setDefaultCommand(driveController());
    }

    private Command driveController() {
        return swerveCommands.driverController(
                () -> xSpeed,
                () -> ySpeed,
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
        return allCommands.getReadyToShoot(() -> speed * MAX_FLYWHEEL_RPM, () -> angle).withName("startShooting");
    }

    public Command startPass() {
        return allCommands.spinIndex().withName("startSpinIndex");
    }

    public Command startIntake() {
        return allCommands.intake().withName("startIntake");
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
        });
    }

    public Command startRotating(RotateDirection direction) {
        return Commands.runOnce(() -> {
            switch (direction) {
                case ClockWise:
                    rotationSpeed = ROTATION_SPEED;
                    break;
                case CounterClockWise:
                    rotationSpeed = -ROTATION_SPEED;
                    break;
            }
        });
    }

    public Command stopShooting() {
        return allCommands.stopFlywheel();
    }

    public Command stopPass() {
        return allCommands.stopIndex();
    }

    public Command stopIntake() {
        return allCommands.stopIntake();
    }

    public Command stopMoving() {
        return Commands.runOnce(() -> {
            xSpeed = 0;
            ySpeed = 0;
        });
    }

    public Command stopRotating() {
        return Commands.runOnce(() -> rotationSpeed = 0);
    }

    public Command stopAll() {
        return Commands.parallel(
            allCommands.stopAll(),
            swerveCommands.stop()
        );
    }

    public Command waitSeconds(double seconds) {
        return Commands.waitSeconds(seconds);
    }

    public Command autoCommand() {
        return Commands.sequence(
            startMoving(MoveDirection.BACK_LEFT),
            waitSeconds(1),
            startShooting(0.1,40),
            startPass()
        );
    }
}
