package frc.robot.Subsystems;

import frc.robot.Constants.constants_Drive;
import frc.robot.Constants.constants_Module;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Module extends SubsystemBase{
    
  //initalize all variables
    public static SparkMax steerMotor;
    public SparkMax driveMotor;
    public final SparkClosedLoopController turningPidController;


    public RelativeEncoder driveMotorEncoder;
    public RelativeEncoder steerMotorEncoder;
    public CANcoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    public final SparkClosedLoopController drivePidController;

    public final SparkMaxConfig driveConfig;
    public final SparkMaxConfig steerConfig;

    public final ClosedLoopConfig driveConfig2;
    public final ClosedLoopConfig steerConfig2;





  //New Swerve Module start
  public Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId, double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {

    //Drive Motor Configs
    driveMotor = new SparkMax(driveNum, MotorType.kBrushless);
    driveMotor.setInverted(invertDrive);

    driveConfig2 = new ClosedLoopConfig();
    driveConfig2.pidf(0.95, 0, 0, 0, ClosedLoopSlot.kSlot0);

    driveConfig = new SparkMaxConfig();
    driveConfig.apply(driveConfig2);
    driveConfig.idleMode(IdleMode.kBrake);
    driveConfig.encoder.positionConversionFactor(1/22.5);
    driveConfig.encoder.velocityConversionFactor(constants_Module.kDriveEncoderRPM2MeterPerSec);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    drivePidController = driveMotor.getClosedLoopController();
    
     

    //Steer Motor Configs
    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);
    steerMotor.setInverted(invertSteer);

    steerConfig2 = new ClosedLoopConfig();
    steerConfig2.pidf(constants_Module.kPTurning, constants_Module.kITurning, constants_Module.kDTurning, constants_Module.kFFTurning, ClosedLoopSlot.kSlot0);
    steerConfig2.positionWrappingEnabled(true);
    steerConfig2.positionWrappingInputRange(720, 1080);

    steerConfig = new SparkMaxConfig();
    steerConfig.apply(steerConfig2);
    steerConfig.idleMode(IdleMode.kBrake);
    steerConfig.encoder.positionConversionFactor(constants_Module.kTurningConversionFactor2Deg);
    steerConfig.encoder.velocityConversionFactor(constants_Module.kTurningEncoderRPM2DegPerSec);

    steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningPidController = steerMotor.getClosedLoopController();


    
    //Absolute Encoder
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    
    //Steer + Drive Motor Encoder
    driveMotorEncoder = driveMotor.getEncoder();
    steerMotorEncoder = steerMotor.getEncoder();
    
    
    //reset encoders after init phase
    resetDrive();
  }



public static SparkClosedLoopController getPIDController() {
  return steerMotor.getClosedLoopController();
}

public void resetDrive() {
  driveMotorEncoder.setPosition(0);
  steerMotorEncoder.setPosition(0);
}
public void resetDriveEncoder() {
  driveMotorEncoder.setPosition(0);
}
//stop method that stops the motors when the stick/s are within the deadzone < 0.01
public void stop() {
  driveMotor.set(0);
  steerMotor.set(0);
}

  public double getAbsoluteEncoderDeg(double AEOffset) {
    double angle = absoluteEncoder.getPosition().getValueAsDouble();
    angle *= 360;
    return (angle  * (absoluteEncoderReversed ? -1 : 1) - AEOffset) % 720;
  }
  
  //Motor calls
  public double getDrivePosition() {
    return driveMotorEncoder.getPosition();
  }
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }
  public double getSteerPosition() {
     return Math.abs(steerMotorEncoder.getPosition() % 720);
  }
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();
  }
  public double getPositionMeters() {
    return driveMotorEncoder.getPosition();
  }
  
 
  
//Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
public SwerveModuleState gState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
}
public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotorEncoder.getPosition(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
}

//This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
//location
public void setDesiredState(SwerveModuleState state) {
  if (Math.abs(state.speedMetersPerSecond) < 0.01) {stop();return;}
  // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(gState().angle.getDegrees()));
  state.cosineScale(gState().angle);
  driveMotor.set(state.speedMetersPerSecond / constants_Drive.kPhysicalMaxSpeedMetersPerSecond);
  turningPidController.setReference(state.angle.getDegrees(), ControlType.kPosition);
}

public void wheelFaceForward(double AEOffset) {
  steerMotorEncoder.setPosition(getAbsoluteEncoderDeg(AEOffset));
  try{Thread.sleep(10);
    turningPidController.setReference(0, ControlType.kPosition);
  }catch (Exception e) {}}


}