package frc.robot.Commands;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.constants_Auto;
import frc.robot.Constants.constants_Drive;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Swerve;

public class Auto {
    
public Drive c_Drive;
public Swerve s_Swerve;
public Limelight s_Limelight;
public PIDController translationConstants = new PIDController(constants_Auto.kPTranslation, constants_Auto.kITranslation, constants_Auto.kDTranslation);
public PIDController rotationConstants = new PIDController(constants_Auto.kPTheta, constants_Auto.kITheta, constants_Auto.kDTheta);
// public Map map






    public Auto(Drive c_Drive, Swerve s_Swerve, Limelight s_Limelight){
        this.c_Drive = c_Drive;
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        // translationConstants.setTolerance(0.1);//meters
        // rotationConstants.setTolerance(10); //maybe degrees?

        // AutoBuilder.configureHolonomic(
        //         s_Swerve::getAutoPose, 
        //         s_Swerve::resetOdometry, 
        //         s_Swerve::getRobotRelativeSpeeds, 
        //         s_Swerve::driveRobotRelative, 
        //         autoConfig, 
        //         s_Swerve::allianceCheck,
        //         s_Swerve
        //         );

        AutoBuilder.configure(
            s_Swerve::getAutoPose,
            s_Swerve::resetOdometry,
            s_Swerve::getRobotRelativeSpeeds,
            s_Swerve::driveRobotRelative,
            pathController,
            robotConfig,
            s_Swerve::allianceCheck,
            s_Swerve);

            
    
                
        NamedCommands.registerCommand("FaceForward Wheels", Commands.runOnce(() -> s_Swerve.faceAllFoward()));
        NamedCommands.registerCommand("AutoDrive", Commands.runOnce(() -> s_Limelight.autoDrive.schedule()));
        // NamedCommands.registerCommand("AutoRunCheck", s_Limelight.autoDriveCheck());

        NamedCommands.registerCommand("AutoDrive Complete", Commands.runOnce(() -> System.out.println("AutoDrive Complete")));
        NamedCommands.registerCommand("AutoDrive Active", Commands.runOnce(() -> System.out.println("AutoDrive Active")));
    }

    // public PathFollowingController pathController = new PPHolonomicDriveController(translationConstants.getP(), translationConstants.getI(), translationConstants.getD(), rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD());

    public PathFollowingController pathController = new PPHolonomicDriveController(
        new com.pathplanner.lib.config.PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
        new com.pathplanner.lib.config.PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()));

    public DCMotor neoV1 = new DCMotor(12, 3, 1.6, 0.4, 471.2389, 1);
    public ModuleConfig moduleConfig = new ModuleConfig(1.5, 4.5, constants_Drive.COF, neoV1, 21, 1);
    public RobotConfig robotConfig = new RobotConfig(25.8, 6.883, moduleConfig, constants_Drive.kTrackWidth, constants_Drive.kWheelBase);
    // public HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
    //     new PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
    //     new PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()), 
    //     constants_Drive.kTeleDriveMaxSpeedMetersPerSecond, 
    //     Constants.ModuleConstants.moduleRadius, 
    //     new ReplanningConfig());


}
