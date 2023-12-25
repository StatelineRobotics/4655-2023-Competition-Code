package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, leftTrigger, rightTrigger;
    private final Supplier<Boolean> fieldOrientedFunction, reverseDirection;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Double> rightTrigger, Supplier<Double> leftTrigger,
            Supplier<Boolean> reverseDirection) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.reverseDirection = reverseDirection;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        double RT = rightTrigger.get();
        double LT = leftTrigger.get();
    

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        RT = Math.abs(RT) > OIConstants.kDeadband ? RT : 0.0;
        LT = Math.abs(LT) > OIConstants.kDeadband ? LT : 0.0;

        double teleMaxSpeed = 2;
        double turningSpeedLimiter = 4;

        teleMaxSpeed = RT > .1 ? 1 : teleMaxSpeed;
        teleMaxSpeed = LT > .1 ? 5 : teleMaxSpeed;
        turningSpeedLimiter = RT > .1 ? 1.5 : turningSpeedLimiter;

        // 3. Make the driving smoother
        xSpeed = reverseDirection.get() ? -(xLimiter.calculate(xSpeed) * (DriveConstants.kPhysicalMaxSpeedMetersPerSecond / teleMaxSpeed))
                                        : (xLimiter.calculate(xSpeed) * (DriveConstants.kPhysicalMaxSpeedMetersPerSecond / teleMaxSpeed));
        ySpeed = reverseDirection.get() ? -(yLimiter.calculate(ySpeed) * (DriveConstants.kPhysicalMaxSpeedMetersPerSecond / teleMaxSpeed))
                                        : (yLimiter.calculate(ySpeed) * (DriveConstants.kPhysicalMaxSpeedMetersPerSecond / teleMaxSpeed));
        turningSpeed = reverseDirection.get() ? -turningLimiter.calculate(turningSpeed)
                                              : turningLimiter.calculate(turningSpeed)
                * (DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / turningSpeedLimiter);

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        } else {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        
        SmartDashboard.putBoolean("field Oriented Drive", fieldOrientedFunction.get());

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
