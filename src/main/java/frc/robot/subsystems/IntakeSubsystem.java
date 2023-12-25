package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;


public class IntakeSubsystem extends SubsystemBase{
    
    private CANSparkMax intakeUpperRollerMotor = new CANSparkMax(MechanismConstants.intakeUpperRollerMotorID, MotorType.kBrushless);
    private CANSparkMax intakeLowerRollerMotor = new CANSparkMax(MechanismConstants.intakeLowerRollerMotorID, MotorType.kBrushless);
    private CANSparkMax intakePositionMotor = new CANSparkMax(MechanismConstants.intakePositionMotorID, MotorType.kBrushless);
    private CANSparkMax conveyorMotor = new CANSparkMax(MechanismConstants.conveyorMotorID, MotorType.kBrushless);
    private AbsoluteEncoder intakePositonEncoder;
    private SparkMaxPIDController intakePidController;
    private DigitalInput input1 = new DigitalInput(8);
    private DigitalInput input2 = new DigitalInput(7);
    private DigitalInput input3 = new DigitalInput(6);

    private DigitalInput gate1 = new DigitalInput(0);
    private DigitalInput gate2 = new DigitalInput(1);

    private RelativeEncoder intakeEncoder;

    private Joystick auxController = new Joystick(1);
    
    public IntakeSubsystem() {
        intakeUpperRollerMotor.setInverted(MechanismConstants.kIntakeUpperRollerMotorInverted);
        intakeLowerRollerMotor.setInverted(MechanismConstants.kIntakeLowerRollerMotorInverted);
        conveyorMotor.setInverted(MechanismConstants.kconveyorMotorInverted);
        intakePositionMotor.restoreFactoryDefaults();
        intakePositionMotor.setSmartCurrentLimit(MechanismConstants.kIntakePositionMotorCurrentLimit);
        intakePositionMotor.setIdleMode(IdleMode.kBrake);
        intakePositionMotor.setInverted(MechanismConstants.kIntakePositionMotorInverted);
        intakePositonEncoder = intakePositionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        intakePidController = intakePositionMotor.getPIDController();  
        intakeEncoder = intakeUpperRollerMotor.getEncoder();

        intakeUpperRollerMotor.setSmartCurrentLimit(20);
        intakeLowerRollerMotor.setSmartCurrentLimit(20);
        intakePositionMotor.setSmartCurrentLimit(40);
        conveyorMotor.setSmartCurrentLimit(20);

        intakeLowerRollerMotor.setIdleMode(IdleMode.kBrake);
        intakeUpperRollerMotor.setIdleMode(IdleMode.kBrake);
    
        // set PID coefficients
        intakePidController.setP(MechanismConstants.kIntakeP);
        intakePidController.setI(MechanismConstants.kIntakeI);
        intakePidController.setD(MechanismConstants.kIntakeD);
        intakePidController.setIZone(MechanismConstants.kIntakeIZ);
        intakePidController.setFF(MechanismConstants.kIntakeFF);
        intakePidController.setOutputRange(MechanismConstants.kIntakeMinOut, MechanismConstants.kIntakeMaxOut);
    
        intakePidController.setFeedbackDevice(intakePositonEncoder);

        intakePositionMotor.burnFlash();
        intakeLowerRollerMotor.burnFlash();
        intakeUpperRollerMotor.burnFlash();
        conveyorMotor.burnFlash();
    }
    
    public double getIntakePosition(){
         return intakePositonEncoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Position", getIntakePosition());
        SmartDashboard.putNumber("IntakeUpperRollerOuput", intakeLowerRollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("IntakeLowerRollerOutput", intakeLowerRollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("ConveyorMotorOutput", conveyorMotor.getAppliedOutput());

        SmartDashboard.putBoolean("Gate 1 Get", gate1.get());
        SmartDashboard.putBoolean("Gate 2 Get", gate2.get());
        SmartDashboard.putBoolean("Center Limit Get", input3.get());

        SmartDashboard.putNumber("Velocity", intakeEncoder.getVelocity());

            //auto stop conveyor and motors
        if(((!input3.get() || !input2.get()) || !input1.get()) && 
            (!auxController.getRawButton(8) && !auxController.getRawButton(9) && !auxController.getRawButton(10))) {
            intakeUpperRollerMotor.set(0);
            intakeLowerRollerMotor.set(0);
            conveyorMotor.set(0);
        }
        /*if(enableAutoGuides) {
            //auto moves conveyor guides   
            if(!gate1.get() && !gate2.get()) {
                closed = true;
            }
            else if(!input1.get() || !input2.get() || !input3.get()) {
                closed = true;
            }
            else if(reset == true) {
                closed = false;
                reset = false;
            }
            conveyorSolenoid.set(!closed);
        }*/
    }

    public Command setPositionCMD(boolean down) {
        if (down) {
            SmartDashboard.putNumber("intakePositionSetPoint", MechanismConstants.kIntakeLowerPostion);
            return 
                    sequence(
                        runOnce(() -> intakeUpperRollerMotor.set(MechanismConstants.kIntakeUpperRollerSpeed)),
                        runOnce(() -> intakeLowerRollerMotor.set(MechanismConstants.kIntakeLowerRollerSpeed)),
                        runOnce(() -> conveyorMotor.set(MechanismConstants.kConveyorSpeed)),
                        runOnce(() -> intakePidController.setReference(MechanismConstants.kIntakeLowerPostion, CANSparkMax.ControlType.kPosition))
                    );
        } 

        else {
            SmartDashboard.putNumber("intakePositionSetPoint", MechanismConstants.kIntakeRaisePostion);
            return
                    sequence(
                        runOnce(() -> intakeUpperRollerMotor.set(0)),
                        runOnce(() -> intakeLowerRollerMotor.set(0)),
                        runOnce(() -> conveyorMotor.set(0)),
                        runOnce(() -> intakePidController.setReference(MechanismConstants.kIntakeRaisePostion, CANSparkMax.ControlType.kPosition))
                    );
        }
    }

    public Command intakeMotorSetCMD(int state) {
        if(state == 1) {    
            return sequence(
                runOnce(() -> intakeLowerRollerMotor.set(-MechanismConstants.kIntakeLowerRollerSpeed)),
                runOnce(() -> intakeUpperRollerMotor.set(-MechanismConstants.kIntakeUpperRollerSpeed)),
                runOnce(() -> conveyorMotor.set(-0.6))
            );
        }
        else if(state == 2) {
            return sequence(
                runOnce(() -> intakeLowerRollerMotor.set(0)),
                runOnce(() -> intakeUpperRollerMotor.set(0)),
                runOnce(() -> conveyorMotor.set(0))
            );
        }
        else if(state == 3) {
            return sequence(
                runOnce(() -> intakeLowerRollerMotor.set(-MechanismConstants.kIntakeLowerRollerSpeed)),
                runOnce(() -> intakeUpperRollerMotor.set(-MechanismConstants.kIntakeUpperRollerSpeed)),
                runOnce(() -> conveyorMotor.set(-1))
            );
        }
        else {
            return sequence(
                runOnce(() -> intakeLowerRollerMotor.set(MechanismConstants.kIntakeLowerRollerSpeed)),
                runOnce(() -> intakeUpperRollerMotor.set(MechanismConstants.kIntakeUpperRollerSpeed)),
                runOnce(() -> conveyorMotor.set(MechanismConstants.kConveyorSpeed))
            );
        }
    }

    public Command conveyorMotorSetCMD(double percent) {
        return runOnce(() -> conveyorMotor.set(percent));
    }

    /*public Command conveyorSolenoidCMD(boolean active) {
        enableAutoGuides = active;
        closed = false;
        return runOnce(() -> conveyorSolenoid.set(active));
    }*/
    
}