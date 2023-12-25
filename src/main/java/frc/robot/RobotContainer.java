package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



public class RobotContainer {

  //private final PneumaticHub m_ph = new PneumaticHub(MechanismConstants.PH_CAN_ID);
  public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final static CommandXboxController driverCommandController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem(armSubsystem);
  private final Joystick auxController = new Joystick(OIConstants.kAuxJoystickPort);
  private final Joystick aux2Controller = new Joystick(OIConstants.kAuxJoystickPort2);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
     () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
     () -> driverController.getRawAxis(OIConstants.kDriverXAxis), 
     () -> driverController.getRawAxis(4), 
     () -> driverController.getRightBumper(),
     () -> driverController.getRawAxis(3),
     () -> driverController.getRawAxis(2),
     () -> driverController.getLeftBumper()));
    
    configureButtonBindings();
}

  private void configureButtonBindings() {
    
    //X For Release Claw
    //POV UP LED Cone
    //POV DOWN LED Cube

    driverCommandController.b().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    driverCommandController.x().onTrue(armSubsystem.setClawStateCMD(true));
    driverCommandController.povLeft().onTrue(swerveSubsystem.setAutoLevel(true)).onFalse(swerveSubsystem.setAutoLevel(false));
    driverCommandController.povRight().onTrue(swerveSubsystem.lockSwerveCMD(true)).onFalse(swerveSubsystem.lockSwerveCMD(false));
    //driverCommandController.povUp().onTrue(new RepeatCommand(blinkinSubsystem.blinkinSetCMD(1)));
    //driverCommandController.povDown().onTrue(new RepeatCommand(blinkinSubsystem.blinkinSetCMD(2)));
  
    //FightStick Command Groups
    new JoystickButton(auxController, OIConstants.kIntakeDownButtonIdx)
                .onTrue(intakeSubsystem.setPositionCMD(true));

    new JoystickButton(auxController, OIConstants.kIntakeUpButtondIdx)
                .onTrue(intakeSubsystem.setPositionCMD(false));

    new JoystickButton(auxController, OIConstants.kArmParkButtonIdx)
                .onTrue(parkAction());

    new JoystickButton(auxController, OIConstants.kArmUpperButtonIdx)
                .onTrue(highNodeAction());  

    new JoystickButton(auxController, OIConstants.kArmLowerButtonIdx)
                .onTrue(midNodeAction());

    new JoystickButton(auxController, OIConstants.kClawClampButtonIdx)
                .onTrue(armSubsystem.setClawStateCMD(false));

    new JoystickButton(auxController, OIConstants.kClawCloseButtonIdx)
                .onTrue(grabAction());
                
    new JoystickButton(auxController, OIConstants.kConveyorReverseIdx)
                .whileTrue(new RepeatCommand(intakeSubsystem.intakeMotorSetCMD(1)))
                .onFalse(intakeSubsystem.intakeMotorSetCMD(0));

    new JoystickButton(auxController, OIConstants.kBrakeIdx)
                .onTrue(armSubsystem.setBrakeStateCMD(true))
                .onFalse(armSubsystem.setBrakeStateCMD(false));

    new JoystickButton(auxController, OIConstants.kIntakeReverseFastIdx)
                .whileTrue(new RepeatCommand(intakeSubsystem.intakeMotorSetCMD(3)))
                .onFalse(intakeSubsystem.intakeMotorSetCMD(0));
    
    /*new JoystickButton(aux2Controller, 1)
                .whileTrue(new RepeatCommand(intakeSubsystem.intakeMotorSetCMD(3)))
                .onFalse(intakeSubsystem.intakeMotorSetCMD(0));*/

    new JoystickButton(auxController, OIConstants.kConveyorRunIdx)
                .whileTrue(intakeSubsystem.intakeMotorSetCMD(0))
                .onFalse(intakeSubsystem.intakeMotorSetCMD(0));

    //Manual Override
    new JoystickButton(aux2Controller, OIConstants.kManualArmPark)
                .onTrue(armSubsystem.setArmPositionCMD(1));

    new JoystickButton(aux2Controller, OIConstants.kManualArmHigh)
                .onTrue(armSubsystem.setArmPositionCMD(2));
                
    new JoystickButton(aux2Controller, OIConstants.kManualArmMid)
                .onTrue(armSubsystem.setArmPositionCMD(3));

    new POVButton(aux2Controller, OIConstants.kManualArmExtendOut)
                .onTrue(armSubsystem.setArmExtendPositionCMD(2));
    
    new POVButton(aux2Controller, OIConstants.kManualArmExtendIn)
                .onTrue(armSubsystem.setArmExtendPositionCMD(1));

    new JoystickButton(aux2Controller, OIConstants.kManualArmExtendGrab)
                .onTrue(armSubsystem.setArmExtendPositionCMD(3));

    new POVButton(aux2Controller, OIConstants.kManualIntakeUp)
                .onTrue(intakeSubsystem.setPositionCMD(false));

    new POVButton(aux2Controller, OIConstants.kManualIntakeDown)
                .onTrue(intakeSubsystem.setPositionCMD(true));

    new JoystickButton(aux2Controller, OIConstants.kManualClawClose)
                .onTrue(armSubsystem.setClawStateCMD(false));

    new JoystickButton(aux2Controller, OIConstants.kManualClawOpen)
                .onTrue(armSubsystem.setClawStateCMD(true));
  }

  public Command grabAction() {
   return sequence(
    armSubsystem.setClawStateCMD(true),
    armSubsystem.setArmExtendPositionCMD(3),
    waitUntil(armSubsystem.getArmExtendGrabController()::atSetpoint),
    parallel(
      waitSeconds(.5),
      armSubsystem.setClawStateCMD(false)
    )
   );
  }

  public Command highNodeAction() {
    return sequence(
      armSubsystem.setArmExtendPositionCMD(1),
      //intakeSubsystem.conveyorSolenoidCMD(false),
      intakeSubsystem.setPositionCMD(true),
      intakeSubsystem.intakeMotorSetCMD(2),
      armSubsystem.setArmPositionCMD(2),
      waitUntil(armSubsystem.getArmController()::atSetpoint),
      armSubsystem.setArmExtendPositionCMD(2)
    );
  }

  public Command highNodeAuto() {
    return sequence(
      armSubsystem.setArmPositionCMD(2),
      intakeSubsystem.setPositionCMD(true),
      waitUntil(armSubsystem.getArmController()::atSetpoint),
      armSubsystem.setArmExtendPositionCMD(2),
      waitUntil(armSubsystem.getArmHighNodeController()::atSetpoint),
      waitUntil(armSubsystem.getArmExtendController()::atSetpoint),
      armSubsystem.setBrakeStateCMD(false),
      waitSeconds(1),
      armSubsystem.setClawStateCMD(true)
    );
  }

  public Command midNodeAction() {
    return sequence(
      armSubsystem.setClawStateCMD(false),
      armSubsystem.setArmExtendPositionCMD(1),
      intakeSubsystem.setPositionCMD(true),
      intakeSubsystem.intakeMotorSetCMD(2),
      armSubsystem.setArmPositionCMD(3)
    );
  }

  public Command parkAction() {
    return sequence(
      armSubsystem.setClawStateCMD(false),
      armSubsystem.setArmPositionCMD(1),
      armSubsystem.setArmExtendPositionCMD(1),
      waitUntil(armSubsystem.getArmHomeController()::atSetpoint),
      armSubsystem.setClawStateCMD(true),
      intakeSubsystem.intakeMotorSetCMD(0)
    );
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public ArmSubsystem getArmSubsystem() {
    return armSubsystem;
  }

  public Command getAutonomousCommand(String selected) {

    //List<PathPlannerTrajectory> path1 = PathPlanner.loadPathGroup("Place Leave Park", new PathConstraints(2, 1));
    //List<PathPlannerTrajectory> path2 = PathPlanner.loadPathGroup("Place and Park", new PathConstraints(2, 1));
    //PathPlannerTrajectory path3 = PathPlanner.loadPath("Place and Park", new PathConstraints(4, 3));

    SmartDashboard.putBoolean("Passed Marker 1", false);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Marker1", new InstantCommand(() -> SmartDashboard.putBoolean("Passed Marker 1", true)));
    eventMap.put("Stop Swerves", new InstantCommand(() -> swerveSubsystem.stopModules()));
    eventMap.put("Level", new InstantCommand(() -> swerveSubsystem.setAutoLevel(true)));

    /*SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(swerveSubsystem::getPose, 
                                                          swerveSubsystem::resetOdometry, 
                                                          DriveConstants.kDriveKinematics, 
                                                          new PIDConstants(1, 0, 0), 
                                                          new PIDConstants(.1, 0, 0),
                                                          swerveSubsystem::setModuleStates,
                                                          eventMap,
                                                          true,
                                                          swerveSubsystem);*/
  
    PathPlannerTrajectory path3 = PathPlanner.loadPath("Test Path", new PathConstraints(4, 4));

    PPSwerveControllerCommand driveCommand5 = new PPSwerveControllerCommand(path3, 
                                                                            swerveSubsystem::getPose,
                                                                            DriveConstants.kDriveKinematics, 
                                                                            new PIDController(0, 0, 0), 
                                                                            new PIDController(0, 0, 0), 
                                                                            new PIDController(.515, 0, 0),
                                                                            swerveSubsystem::setModuleStates,
                                                                            true,
                                                                            swerveSubsystem);

    FollowPathWithEvents fullCommand = new FollowPathWithEvents(driveCommand5, path3.getMarkers(), eventMap);

    return fullCommand;

    /*if(selected.equals("Place and Park")) {
      PPSwerveControllerCommand driveCommand1 = new PPSwerveControllerCommand(path2.get(0), 
                                                                              swerveSubsystem::getPose,
                                                                              DriveConstants.kDriveKinematics, 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.515, 0, 0),
                                                                              swerveSubsystem::setModuleStates,
                                                                              true,
                                                                              swerveSubsystem);

      //FollowPathWithEvents command = new FollowPathWithEvents(driveCommand1, path3.getMarkers(), eventMap);

      
      return sequence(
        intakeSubsystem.setPositionCMD(true),
        intakeSubsystem.conveyorMotorSetCMD(0),
        waitSeconds(2),
        grabAction(),
        waitSeconds(.5),
        highNodeAuto(),
        waitSeconds(2),
        parkAction(),
        waitUntil(armSubsystem.getArmHomeController()::atSetpoint),
        intakeSubsystem.setPositionCMD(false),
        waitSeconds(2),
        driveCommand1
      );
    }

    else {
      PPSwerveControllerCommand driveCommand1 = new PPSwerveControllerCommand(path1.get(0), 
                                                                              swerveSubsystem::getPose,
                                                                              DriveConstants.kDriveKinematics, 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.515, 0, 0),
                                                                              swerveSubsystem::setModuleStates,
                                                                              true,
                                                                              swerveSubsystem);

      PPSwerveControllerCommand driveCommand2 = new PPSwerveControllerCommand(path1.get(1), 
                                                                              swerveSubsystem::getPose,
                                                                              DriveConstants.kDriveKinematics, 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.01, 0, 0), 
                                                                              new PIDController(.53, 0, 0),
                                                                              swerveSubsystem::setModuleStates,
                                                                              true,
                                                                              swerveSubsystem);
                                                                              
      return sequence(
        intakeSubsystem.setPositionCMD(true),
        intakeSubsystem.conveyorMotorSetCMD(0),
        waitSeconds(1.5),
        grabAction(),
        waitSeconds(.5),
        highNodeAuto(),
        waitSeconds(2),
        parkAction(),
        waitUntil(armSubsystem.getArmHomeController()::atSetpoint),
        intakeSubsystem.setPositionCMD(false),
        waitSeconds(2),
        driveCommand1,
        waitSeconds(3),
        driveCommand2,
        swerveSubsystem.setAutoLevel(true)
      );      
    }  */    
      //Command parkAndPlace = autoBuilder.fullAuto(path1);
     }
}
