package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.constants_Module;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Module extends SubsystemBase{
    
  //initalize all variables
  // public SparkMax driveMotor;
  // public RelativeEncoder driveMotorEncoder;
  // public final SparkClosedLoopController drivePidController;
  // public final SparkMaxConfig driveConfig;
  // public final ClosedLoopConfig driveConfig2;
  TalonFX driveMotor; 
  VelocityVoltage velocityRequest;
  MotionMagicVelocityVoltage motionMagicRequest;
  TalonFXConfiguration driveConfig;
  NeutralOut neutralOut;

  static SparkMax steerMotor;
  final SparkClosedLoopController turningPidController;
  RelativeEncoder steerMotorEncoder;
  CANcoder absoluteEncoder;
  boolean absoluteEncoderReversed;
  final ClosedLoopConfig steerConfig2;
  final SparkMaxConfig steerConfig;

  double desiredDriveVelocity = 0;


  //New Swerve Module start
  public Module(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer, int absoluteEncoderId, double absoluteEncoderOffsetRad, Boolean absoluteEncoderReversed) {

    //Drive Motor Configs
    // driveMotor = new SparkMax(driveNum, MotorType.kBrushless);
    // driveMotor.setInverted(invertDrive);

    // driveConfig2 = new ClosedLoopConfig();
    // driveConfig2.pidf(0.95, 0, 0, 0, ClosedLoopSlot.kSlot0);

    // driveConfig = new SparkMaxConfig();
    // driveConfig.apply(driveConfig2);
    // driveConfig.idleMode(IdleMode.kBrake);
    // driveConfig.encoder.positionConversionFactor(1/22.5);
    // driveConfig.encoder.velocityConversionFactor(constants_Module.kDriveEncoderRPM2MeterPerSec);

    // driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // drivePidController = driveMotor.getClosedLoopController();
    
     driveMotor = new TalonFX(driveNum, "rio");
     driveConfig = new TalonFXConfiguration();
     velocityRequest = new VelocityVoltage(0).withSlot(0);
     motionMagicRequest = new MotionMagicVelocityVoltage(0);

     driveConfig.Slot0.kS = Constants.constants_Module.kS_Drive;
     driveConfig.Slot0.kV = Constants.constants_Module.kV_Drive;
     driveConfig.Slot0.kA = Constants.constants_Module.kA_Drive;
     driveConfig.Slot0.kP = Constants.constants_Module.kP_Drive;
     driveConfig.Slot0.kI = Constants.constants_Module.kI_Drive;
     driveConfig.Slot0.kD = Constants.constants_Module.kD_Drive;

     driveConfig.MotionMagic.MotionMagicAcceleration = 400;
     driveConfig.MotionMagic.MotionMagicJerk = 4000;

     driveMotor.getConfigurator().apply(driveConfig);


    //Steer Motor Configs
    steerMotor = new SparkMax(steerNum, MotorType.kBrushless);

    steerConfig2 = new ClosedLoopConfig();
    steerConfig2.pidf(constants_Module.kPTurning, constants_Module.kITurning, constants_Module.kDTurning, constants_Module.kFFTurning, ClosedLoopSlot.kSlot0);
    steerConfig2.positionWrappingEnabled(true);
    steerConfig2.positionWrappingInputRange(720, 1080);

    steerConfig = new SparkMaxConfig();
    steerConfig.apply(steerConfig2);
    steerConfig.idleMode(IdleMode.kBrake);
    steerConfig.encoder.positionConversionFactor(constants_Module.kTurningConversionFactor2Deg);
    steerConfig.encoder.velocityConversionFactor(constants_Module.kTurningEncoderRPM2DegPerSec);
    steerConfig.inverted(invertSteer);

    steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningPidController = steerMotor.getClosedLoopController();


    
    //Absolute Encoder
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);
    
    //Steer + Drive Motor Encoder
    // driveMotorEncoder = driveMotor.getEncoder();
    steerMotorEncoder = steerMotor.getEncoder();
    
    
    //reset encoders after init phase
    resetDrive();
  }

  public void resetDrive() {
    driveMotor.setPosition(0);
    steerMotorEncoder.setPosition(0);
  }
  public void resetDriveEncoder() {
    driveMotor.setPosition(0);

  }
  //stop method that stops the motors when the stick/s are within the deadzone < 0.01
  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public double getAbsoluteEncoderDeg(double AEOffset) {
    // double angle = absoluteEncoder.getPosition().getValueAsDouble();
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 360;
    return (angle  * (absoluteEncoderReversed ? -1 : 1) - AEOffset) % 720;
  }
  
  //Motor calls
  public double getSteerPosition() {
    return Math.abs(steerMotorEncoder.getPosition() % 720);
  }
  public double getSteerVelocity() {
    return steerMotorEncoder.getVelocity();
  }
  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }
  public double getPositionMeters() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public void getUpToSpeed(double velocity)
  {
    if(velocity <= 0.01)
    {
      setDriveNeutralOutput();
    }else
    {
      driveMotor.setControl(motionMagicRequest.withVelocity(velocity));
    }
  }

  public void setDesiredVelocity(double driveSpeed)
  {
    this.desiredDriveVelocity = driveSpeed;
  }

  public void setDriveNeutralOutput()
  {
    driveMotor.setControl(new NeutralOut()); //TODO see if this is coast or brake
  }
  
  
  //Creating the current state of the modules. A drive velo and an angle are needed. We use an off set of -90 for the angle
  public SwerveModuleState gState() 
  {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
  }
  public SwerveModulePosition getPosition() 
  {
    return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), Rotation2d.fromDegrees(steerMotorEncoder.getPosition()));
  }

  //This is our setDesiredState alg. Takes the current state and the desired state shown by the controller and points the wheels to that 
  //location
  public void setDesiredState(SwerveModuleState state)
  {
    // state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(gState().angle.getDegrees()));
    state.cosineScale(gState().angle);
    getUpToSpeed(state.speedMetersPerSecond);
    turningPidController.setReference(state.angle.getDegrees(), ControlType.kPosition);
  }

  public void wheelFaceForward(double AEOffset)
  {
    steerMotorEncoder.setPosition(getAbsoluteEncoderDeg(AEOffset));
    try{Thread.sleep(10);
      turningPidController.setReference(0, ControlType.kPosition);
    }catch (Exception e) {}
  }

}