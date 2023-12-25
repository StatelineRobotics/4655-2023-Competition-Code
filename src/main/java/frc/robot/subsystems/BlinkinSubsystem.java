package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class BlinkinSubsystem extends SubsystemBase{

    private Spark blinkin = new Spark(MechanismConstants.kBlinkinID);
    private XboxController driveController = new XboxController(0);
    private ArmSubsystem armSubsystem;
    private int state;

    public BlinkinSubsystem(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }


    @Override
    public void periodic() {
        if(driveController.getPOV() == 0) {
            state = 1;
        }
        else if(driveController.getPOV() == 180) {
            state = 2;
        }


        if(state == 1) {
            blinkin.set(0.15);
        }
        else if(state == 2) {
            blinkin.set(0.35);
        } 
        else {
            blinkin.set(.43);
        }
    }
}
