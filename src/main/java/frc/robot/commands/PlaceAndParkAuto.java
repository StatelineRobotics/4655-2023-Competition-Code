package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

public class PlaceAndParkAuto extends SequentialCommandGroup{

    public PlaceAndParkAuto(RobotContainer robotContainer) {

        SwerveSubsystem swerveSubsystem = robotContainer.getSwerveSubsystem();
        IntakeSubsystem intakeSubsystem = robotContainer.getIntakeSubsystem();
        ArmSubsystem armSubsystem = robotContainer.getArmSubsystem();

        PathPlannerTrajectory path = PathPlanner.loadPath("Place and Park", new PathConstraints(3, 3));

        PPSwerveControllerCommand driveCommand1 = new PPSwerveControllerCommand(path, 
                                                                            swerveSubsystem::getPose,
                                                                            DriveConstants.kDriveKinematics, 
                                                                            new PIDController(0, 0, 0), 
                                                                            new PIDController(.0, 0, 0), 
                                                                            new PIDController(.515, 0, 0),
                                                                            swerveSubsystem::setModuleStates,
                                                                            true,
                                                                            swerveSubsystem);
                                                                            

        addCommands(
            armSubsystem.setBrakeStateCMD(true),
            waitSeconds(.25),
            robotContainer.highNodeAuto(),
            waitSeconds(.5),
            armSubsystem.setBrakeStateCMD(false),
            robotContainer.parkAction(),
            intakeSubsystem.setPositionCMD(false),
        //driveCommand1,
            new SwerveAutoLevelCmd(swerveSubsystem)
        );
    }
}
