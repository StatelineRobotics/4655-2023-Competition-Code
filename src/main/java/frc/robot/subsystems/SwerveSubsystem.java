package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningMotorReversed,
        DriveConstants.kFrontLeftDriveabsoluteEncoderOffset);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningMotorReversed,
        DriveConstants.kBackLeftDriveabsoluteEncoderOffset);
 
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningMotorReversed,
        DriveConstants.kFrontRightDriveabsoluteEncoderOffset);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningMotorReversed,
        DriveConstants.kBackRightDriveabsoluteEncoderOffset);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private boolean zeroSwerves = false;
    private boolean autoLevel = false;
    private boolean lockSwerves = false;

    private double gyroVal;

    private XboxController driveController = new XboxController(0);
   
    
    // Look at this
    
    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            });

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e){
            } 
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Command setAutoLevel(boolean state) {
    return runOnce(() -> autoLevel = state);
  }

  public Command lockSwerveCMD(boolean state) {
    return runOnce(() -> lockSwerves = state);
  }

  
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);    
  }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Robot Heading", getHeading());;
        SmartDashboard.putNumber("FL Angle", frontLeft.getAbsCurrentAngle());
        SmartDashboard.putNumber("FR Angle", frontRight.getAbsCurrentAngle());
        SmartDashboard.putNumber("BL Angle", backLeft.getAbsCurrentAngle());
        SmartDashboard.putNumber("BR Angle", backRight.getAbsCurrentAngle());
        SmartDashboard.putNumber("Robot Pos X", getPose().getX());
        SmartDashboard.putNumber("Robot Pos Y", getPose().getY());
        SmartDashboard.putNumber("Robot Pos Z", gyro.getAngle());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putBoolean("Auto Level", autoLevel);

        SmartDashboard.putNumber("Front Left Output", frontLeft.getDriveMotor().get());
        
        // Update the odometry in the periodic block
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
            });

        gyroVal = gyro.getRoll();
    }

    public double getGyroValue() {
        return gyro.getRoll();
    }
    
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        if(driveController.getStartButton()) {
            zeroSwerves = true;
        }
        else {
            zeroSwerves = false;
        }

        if(driveController.getYButton()) {
            autoLevel = true;
        }
        else {
            autoLevel = false;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], zeroSwerves, autoLevel, gyroVal, lockSwerves);
        frontRight.setDesiredState(desiredStates[1], zeroSwerves, autoLevel, gyroVal, lockSwerves);
        backLeft.setDesiredState(desiredStates[2], zeroSwerves, autoLevel, gyroVal, lockSwerves);
        backRight.setDesiredState(desiredStates[3], zeroSwerves, autoLevel, gyroVal, lockSwerves);
    }

    //LOOK at this.
    /** Resets the drive encoders to currently read a position of 0. */
 // public void resetEncoders() {
 //  frontLeft.resetEncoders();
 //   backLeft.resetEncoders();
 //   frontRight.resetEncoders();
 //   backRight.resetEncoders();
 // }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
 // public double getTurnRate() {
 //   return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
 // }

}
