package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAutoLevelCmd extends CommandBase {
    
    private SwerveSubsystem swerveSubsystem;
    private PIDController balanceController = new PIDController(.04, 0, 0);
    private boolean finished;

    public SwerveAutoLevelCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        finished = false;
        balanceController.setTolerance(1.5);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double balanceSpeed = balanceController.calculate(swerveSubsystem.getGyroValue(), 0);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(balanceSpeed, 0, 0);

        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

        SmartDashboard.putBoolean("Auto Level", true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
