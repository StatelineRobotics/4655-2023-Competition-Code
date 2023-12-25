package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class ArmSubsystem extends SubsystemBase {
   
    private CANSparkMax rightArmMotor = new CANSparkMax(MechanismConstants.rightArmMotorID, MotorType.kBrushless);
    private CANSparkMax leftArmMotor = new CANSparkMax(MechanismConstants.leftArmMotorID, MotorType.kBrushless);
    private CANSparkMax armExtendMotor = new CANSparkMax(MechanismConstants.armExtendMotorID, MotorType.kBrushless);
    private AbsoluteEncoder armEncoder;
    private RelativeEncoder armExtendEncoder;
    private SparkMaxPIDController armPidController;
    private SparkMaxPIDController armExtendPidController;
    private PneumaticHub m_ph = new PneumaticHub(MechanismConstants.PH_CAN_ID);
    private Solenoid clawSolenoid;
    private Solenoid brakeSolenoid;
    private SparkMaxLimitSwitch extendLimitSwitch;
    private PIDController armAutoPidController;
    private PIDController armExtendGrabAutoPidController;
    private PIDController armHomePidController;
    private PIDController armExtendAutoPidController;
    private PIDController armHighNodeController;


    public ArmSubsystem() {
        rightArmMotor.restoreFactoryDefaults();
        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.setInverted(MechanismConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(MechanismConstants.kLeftArmMotorInverted);
        armExtendMotor.setInverted(MechanismConstants.kLeftArmMotorInverted);
        rightArmMotor.setSmartCurrentLimit(MechanismConstants.kArmMotorCurrentLimit);
        leftArmMotor.setSmartCurrentLimit(MechanismConstants.kArmMotorCurrentLimit);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setIdleMode(IdleMode.kBrake);
        armExtendMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setSmartCurrentLimit(40);
        rightArmMotor.setSmartCurrentLimit(40);
        armExtendMotor.setSmartCurrentLimit(10);

        leftArmMotor.follow(rightArmMotor);
        armEncoder = rightArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        armEncoder.setInverted(false);
        armEncoder.setZeroOffset(0.68);
        armExtendEncoder = armExtendMotor.getEncoder();
        armPidController = rightArmMotor.getPIDController();
        armExtendPidController = armExtendMotor.getPIDController();
        

        // set PID coefficients
        armPidController.setP(MechanismConstants.kArmP);
        armPidController.setI(MechanismConstants.kArmI);
        armPidController.setD(MechanismConstants.kArmD);
        armPidController.setIZone(MechanismConstants.kArmIZ);
        armPidController.setFF(MechanismConstants.kArmFF);
        armPidController.setOutputRange(MechanismConstants.kArmMinOut, MechanismConstants.kArmMaxOut);
        armPidController.setFeedbackDevice(armEncoder);

        armExtendPidController.setP(MechanismConstants.kArmExtendP);
        armExtendPidController.setI(MechanismConstants.kArmExtendI);
        armExtendPidController.setD(MechanismConstants.kArmExtendD);
        armExtendPidController.setIZone(MechanismConstants.kArmExtendIZ);
        armExtendPidController.setFF(MechanismConstants.kArmExtendFF);
        armExtendPidController.setOutputRange(MechanismConstants.kArmExtendMinOut, MechanismConstants.kArmExtendMaxOut);
        armExtendPidController.setFeedbackDevice(armExtendEncoder);
        extendLimitSwitch = armExtendMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        extendLimitSwitch.enableLimitSwitch(true);
        //armExtendEncoder.setPosition(0);

        armAutoPidController = new PIDController(MechanismConstants.kArmP, MechanismConstants.kArmI, MechanismConstants.kArmD);
        armAutoPidController.setTolerance(.1);

        armHomePidController = new PIDController(MechanismConstants.kArmP, MechanismConstants.kArmI, MechanismConstants.kArmD);
        armHomePidController.setTolerance(.01);

        armExtendAutoPidController = new PIDController(MechanismConstants.kArmP, MechanismConstants.kArmI, MechanismConstants.kArmD);
        armExtendAutoPidController.setTolerance(.5);

        armExtendGrabAutoPidController = new PIDController(MechanismConstants.kArmP, MechanismConstants.kArmI, MechanismConstants.kArmD);
        armExtendGrabAutoPidController.setTolerance(3);

        armHighNodeController = new PIDController(MechanismConstants.kArmP, MechanismConstants.kArmI, MechanismConstants.kArmD);
        armHighNodeController.setTolerance(.05);


        rightArmMotor.burnFlash();
        leftArmMotor.burnFlash();
        armExtendMotor.burnFlash();

        // set Pneumatic Hub
        
        clawSolenoid = m_ph.makeSolenoid(MechanismConstants.clawSolenoidID);
        brakeSolenoid = m_ph.makeSolenoid(MechanismConstants.brakeSolenoidID);
    }
    public double getArmPosition(){
        return armEncoder.getPosition();
   }
 
   public double getArmExtendPosition(){
    return armExtendEncoder.getPosition();
    }

    public BooleanSupplier extendOk() {
        if(armEncoder.getPosition() > .6) {
            return () -> true;
        }
        else {
            return () -> false;
        }
    }

    public boolean getClawState() {
        return clawSolenoid.get();
    }

   @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putNumber("Arm Extend Position", getArmExtendPosition());
        SmartDashboard.putNumber("RightArmOuput", rightArmMotor.getAppliedOutput());
        SmartDashboard.putNumber("LeftArmOutput", leftArmMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Extend Limit Switch", extendLimitSwitch.isPressed());
        SmartDashboard.putNumber("Extend Motor Temp", armExtendMotor.getMotorTemperature());
      
        if(extendLimitSwitch.isPressed()) {
            armExtendEncoder.setPosition(0);
        }

        armAutoPidController.calculate(getArmPosition(), .55);
        armExtendGrabAutoPidController.calculate(getArmExtendPosition(), 2);
        armHomePidController.calculate(getArmPosition(), .12);
        armExtendAutoPidController.calculate(getArmExtendPosition(), 112);
        armHighNodeController.calculate(getArmPosition(), .75);

        //if(armEncoder.getPosition() < .6) {
        //    armExtendPidController.setReference(getArmExtendPosition(), CANSparkMax.ControlType.kPosition);
        //}
    }

    public Command setArmExtendPositionCMD(int armExtendDesiredPosition) {
        if (armExtendDesiredPosition == 1) {
            SmartDashboard.putNumber("ArmExtendPositionSetPoint", MechanismConstants.kArmExtendInPostion);
            return runOnce(() -> armExtendPidController.setReference(MechanismConstants.kArmExtendInPostion, CANSparkMax.ControlType.kPosition));
        } 
        else if (armExtendDesiredPosition == 2) {
            SmartDashboard.putNumber("ArmExtendPositionSetPoint", MechanismConstants.kArmExtendHighPostion);
            return runOnce(() -> armExtendPidController.setReference(MechanismConstants.kArmExtendHighPostion, CANSparkMax.ControlType.kPosition));
        }
        else if (armExtendDesiredPosition == 3) {
            SmartDashboard.putNumber("ArmExtendPositionSetPoint", MechanismConstants.kArmExtendGrabPostion);
            return runOnce(() -> armExtendPidController.setReference(MechanismConstants.kArmExtendGrabPostion, CANSparkMax.ControlType.kPosition));
        }
        else  {
            SmartDashboard.putNumber("ArmExtendPositionSetPoint", 999);
            return runOnce(() -> armExtendPidController.setReference(getArmExtendPosition(), CANSparkMax.ControlType.kPosition));
        }
    }

    public Command setArmPositionCMD(int armDesiredPosition) {
        if (armDesiredPosition == 1) {
            SmartDashboard.putNumber("ArmPositionSetPoint", MechanismConstants.kArmParkPostion);
            return runOnce(() -> armPidController.setReference(MechanismConstants.kArmParkPostion, CANSparkMax.ControlType.kPosition));
        } 
        else if (armDesiredPosition == 2) {
            SmartDashboard.putNumber("ArmPositionSetPoint", MechanismConstants.kArmUpperPostion);
            return runOnce(() -> armPidController.setReference(MechanismConstants.kArmUpperPostion, CANSparkMax.ControlType.kPosition));
        } 
        else if (armDesiredPosition == 3) {
            SmartDashboard.putNumber("ArmPositionSetPoint", MechanismConstants.kArmLowerPostion);
            return runOnce(() -> armPidController.setReference(MechanismConstants.kArmLowerPostion, CANSparkMax.ControlType.kPosition));
        }
        else {
            return runOnce(() -> armPidController.setReference(getArmPosition(), CANSparkMax.ControlType.kPosition));
        }
    }

    public Command setClawStateCMD(boolean clawState) {
        return runOnce(() -> clawSolenoid.set(clawState));
    }

    public Command setBrakeStateCMD(boolean state) {
        return runOnce(() -> brakeSolenoid.set(state));
    }   

    public PIDController getArmController() {
        return armAutoPidController;
    }

    public PIDController getArmExtendController() {
        return armExtendAutoPidController;
    }

    public PIDController getArmExtendGrabController() {
        return armExtendGrabAutoPidController;
    }

    public PIDController getArmHomeController() {
        return armHomePidController;
    }

    public PIDController getArmHighNodeController() {
        return armHighNodeController;
    }

}
