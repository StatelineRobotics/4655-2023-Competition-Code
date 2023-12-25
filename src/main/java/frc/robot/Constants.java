// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;  //was 5.6463
    public static final double kTurningMotorGearRatio = 1 / 21.4285714; //was 18.0 
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MetersPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2MetersPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.75;
  }

  public static class DriveConstants {
    //distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    
    //distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftDriveMotorPort = 7;
    public static final int kBackLeftTurningMotorPort = 8;
    public static final int kBackRightDriveMotorPort = 5;
    public static final int kBackRightTurningMotorPort = 6;


    public static final boolean kFrontLeftTurningMotorReversed = false;
    public static final boolean kBackLeftTurningMotorReversed = false;
    public static final boolean kFrontRightTurningMotorReversed = false;
    public static final boolean kBackRightTurningMotorReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;



    public static final double kFrontLeftDriveabsoluteEncoderOffset = -425.56;
    public static final double kBackLeftDriveabsoluteEncoderOffset = -679.83; // -713.96, 34.13
    public static final double kFrontRightDriveabsoluteEncoderOffset = -477.99;
    public static final double kBackRightDriveabsoluteEncoderOffset = -821.26;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4196;  //Look at this
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    //public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;  //was 4
    //public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;  //was 4
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final Boolean kGyroReversed = false;
  }

  public static final class MechanismConstants {
    public static final int intakeUpperRollerMotorID = 11;
    public static final int intakeLowerRollerMotorID = 12;
    public static final int intakePositionMotorID = 10;
    public static final int conveyorMotorID = 9;
    public static final int rightArmMotorID = 14;
    public static final int leftArmMotorID = 13;
    public static final int armExtendMotorID = 15;
    public static final int PH_CAN_ID = 16;
    public static final int clawSolenoidID = 0;
    public static final int conveyorSolenoidID = 1;
    public static final int brakeSolenoidID = 1;
  
    
    public static final boolean kIntakeUpperRollerMotorInverted = true;
    public static final boolean kIntakeLowerRollerMotorInverted = false;
    public static final boolean kIntakePositionMotorInverted = false;
    public static final boolean kconveyorMotorInverted = true;
    public static final boolean kRightArmMotorInverted = true;
    public static final boolean kLeftArmMotorInverted = true;
    public static final boolean kArmExtendMotorInverted = true;

    public static final double kIntakeUpperRollerSpeed = .45; //was .60
    public static final double kIntakeLowerRollerSpeed = .5625; //was .75
    public static final double kConveyorSpeed = .75;
    public static final double kIntakeRaisePostion = .4;
    public static final double kIntakeLowerPostion = .565;
    public static final int kIntakePositionMotorCurrentLimit = 30;
    public static final double kIntakeP = 1.5;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kIntakeIZ = 0;
    public static final double kIntakeFF = 0;
    public static final double kIntakeMinOut = -.75; //was -.5
    public static final double kIntakeMaxOut = 0.25; 

    public static final double kArmParkPostion = .1125;
    public static final double kArmLowerPostion = .805;
    public static final double kArmUpperPostion = .765;
    public static final int kArmMotorCurrentLimit = 40;
    public static final double kArmP = 4; //was 1.5
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmIZ = 0;
    public static final double kArmFF = 0;
    public static final double kArmMinOut = -0.25;
    public static final double kArmMaxOut = 0.5;

  
    public static final double kArmExtendHighPostion = 112;
    public static final double kArmExtendInPostion = -10;
    public static final double kArmExtendGrabPostion = 2;
    public static final int kArmExtendMotorCurrentLimit = 40;
    public static final double kArmExtendP = .5; //was 1
    public static final double kArmExtendI = 0;
    public static final double kArmExtendD = 0;
    public static final double kArmExtendIZ = 0;
    public static final double kArmExtendFF = 0;
    public static final double kArmExtendMinOut = -.9;
    public static final double kArmExtendMaxOut = 1;

    public static final int kBlinkinID = 0;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
}

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxJoystickPort = 1;
    public static final int kAuxJoystickPort2 = 2;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 3;

    public static final int kIntakeDownButtonIdx = 1; 
    public static final int kIntakeUpButtondIdx = 13; 

    public static final int kArmParkButtonIdx = 4;
    public static final int kArmUpperButtonIdx = 6;
    public static final int kArmLowerButtonIdx = 5;
    public static final int kClawClampButtonIdx = 3;
    public static final int kClawCloseButtonIdx = 2;
    public static final int kConveyorRunIdx = 8;
    public static final int kConveyorReverseIdx = 10;
    public static final int kBrakeIdx = 7;
    public static final int kIntakeReverseFastIdx = 9;


    //AuxController2
    public static final int kManualArmPark = 1;
    public static final int kManualArmHigh = 3;
    public static final int kManualArmMid = 4;

    public static final int kManualArmExtendGrab = 2;
    public static final int kManualArmExtendOut = 90;
    public static final int kManualArmExtendIn = 270;

    public static final int kManualIntakeDown = 180;
    public static final int kManualIntakeUp = 0;

    public static final int kManualClawClose = 6;
    public static final int kManualClawOpen = 5;



    

    
    public static final double kDeadband = 0.10;
  }
}
