// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class constants_Drive {

    public static final Measure<DistanceUnit> wheelRadius = edu.wpi.first.units.Units.Inches.of(1.5);
    // public static final double wheelRadius = Units.inchesToMeters(1.5);
    public static final double COF = 1.2;
    public static final double kTrackWidth = Units.inchesToMeters(23.75);
      // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23.75);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); //back right


 
    //start front front left to front right to back right and all drives then all steers then all absolutes
    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
    
    public static final int kBackRightTurningMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 3;
    public static final int kBackRightDriveAbsoluteEncoderPort = 3;

    public static final int kBackLeftTurningMotorPort = 4;
    public static final int kBackLeftDriveMotorPort = 4;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 4;

    
    //TODO Test and input all module offsets
    public static final double kFLDegrees = 0;
    public static final double kFRDegrees = 0;
    public static final double kBLDegrees = 0;
    public static final double kBRDegrees = 0;


    //TODO Invert any motor to match controller output
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 6.949; //6.949 for Swerve X, 4.60248 for sd
    public static final LinearVelocityUnit kMaxTestedSpeed = LinearVelocityUnit.combine(edu.wpi.first.units.Units.Meters.of(4.5).unit(), edu.wpi.first.units.Units.Seconds.of(1).unit());
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =kPhysicalMaxSpeedMetersPerSecond/(kTrackWidth/2);

    //For limiting speed while driving
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.0;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2.0;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.75;
  }
  
  public static final class constants_Module {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 4.59 / 1; //4.59 for Swerve X, 6.75 for sds
    public static final double kTurningMotorGearRatio = 13.3714 / 1; //13.3714 for Swerve X, 12.8 for sds
    public static final double kDriveEncoderRot2Meter = 1/16.0344; //Not sure try 1/16.0344, 1/23.58 for sds
    
    public static final double kTurningConversionFactor2Deg =  28.25;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2DegPerSec = kTurningConversionFactor2Deg / 60;

    public static final double kPTurning = 0.0075;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.75;
    public static final double kFFTurning = 0;

    public static final double kS_Drive = 0.4;
    public static final double kV_Drive = 0.124;
    public static final double kA_Drive = 0.1;
    public static final double kP_Drive = 0.1;
    public static final double kI_Drive = 0;
    public static final double kD_Drive = 0.1;

    public static final double moduleRadius = Units.inchesToMeters(Constants.constants_Drive.kTrackWidth/2); //measured from center of robot to furthest module.
  }

  public static final class constants_OI {
    public static final int kOPControllerPort = 0;
    public static final double kDeadband = 0.09;
    public static final int kLeftStickPort = 1;
    public static final int kRightStickPort = 2;
  }
 

  
  public static final class constants_AprilTags{ //[id num, height in inches, coordinate x, coordinate y, heading]
    public static final double[] blueSourceRight = {1, 53.375};
    public static final double[] blueSourceLeft = {2, 53.375};
    public static final double[] redSpeakerRight = {3, 57.125};
    public static final double[] redSpeakerLeft = {4, 57.125};
    public static final double[] redAmp = {5, 53.25};
    public static final double[] blueAmp = {6, 53.25};
    public static final double[] blueSpeakerRight = {7, 57.125};
    public static final double[] blueSpeakerLeft = {8, 57.125};
    public static final double[] redSourceRight = {9, 53.375};
    public static final double[] redSourceLeft = {10, 53.375};
    public static final double[] redStageSource = {11, 51.25};
    public static final double[] redStageAmp = {12, 51.25};
    public static final double[] redStageCenter = {13, 51.25};
    public static final double[] blueStageCenter = {14, 51.25};
    public static final double[] blueStageAmp = {15, 51.25};
    public static final double[] blueStageSource = {16, 51.25};
  }

  public static final class constants_Limelight{
    public static final double thetakP = 4;
    public static final double thetakI = 0.0002;
    public static final double thetakD = 0;

    public static final double linearkP = 1.25;
    public static final double linearkI = 0.001;
    public static final double linearkD = 0.05;

    public static final double Angle_Coral = -15;
    public static final double DistanceForward_Coral = 5; //inches
    public static final double DistanceRight_Coral = -3; //inches
    public static final double Height_Coral = 23.75; //inches

    public static final double Angle_Tags = 15;
    public static final double DistanceForward_Tags = 3; //inches
    public static final double DistanceRight_Tags = -3; //inches
    public static final double Height_Tags = 24.5; //inches
  }


  public static final class constants_Auto {
    public static final double kMaxSpeedMetersPerSecond = constants_Drive.kPhysicalMaxSpeedMetersPerSecond/2;//0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = constants_Drive.kTeleDriveMaxAccelerationUnitsPerSecond/2;//0.25;
    public static final double kMaxAngularSpeedRadiansPerSecond =  constants_Drive.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double kMaxAngularAccelerationUnitsPerSecond = constants_Drive.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

    public static  double kPTranslation = 3.0;
    public static  double kITranslation = 0.1;
    public static  double kDTranslation = 2;

    public static final double kPTheta = 4.5;
    public static final double kITheta = 0.1;
    public static final double kDTheta = 0;


    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationUnitsPerSecond);
    public static final TrapezoidProfile.Constraints kLinearConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared
            );
  }
}

