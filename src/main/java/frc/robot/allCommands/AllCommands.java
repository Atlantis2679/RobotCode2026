package frc.robot.allCommands;

import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodCommands;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.roller.RollerCommands;
import frc.robot.subsystems.intake.slapdown.Slapdown;
import frc.robot.subsystems.intake.slapdown.SlapdownCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;



public class AllCommands {
    private Slapdown slapdown;
    private Roller roller;
    private FlyWheel flyWheel;
    private Hood hood;
    private Swerve swerve;
    
    private SlapdownCommands slapdownCMDs;
    private RollerCommands rollerCMDs;
    private FlyWheelCommands flyWheelCommands;
    private HoodCommands hoodCMDs;
    private SwerveCommands swerveCMDs;

    public AllCommands(Slapdown slapdown, Roller roller, FlyWheel flyWheel, Hood hood, Swerve swerve){
        this.slapdown = slapdown;
        this.roller = roller;
        this.flyWheel = flyWheel;
        this.hood = hood;
        this.swerve = swerve;

        slapdownCMDs = new SlapdownCommands(slapdown);
        rollerCMDs = new RollerCommands(roller);
        flyWheelCommands = new FlyWheelCommands(flyWheel);
        hoodCMDs = new HoodCommands(hood);
        swerveCMDs = new SwerveCommands(swerve);
    }
}