package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;

    private final double absoluteEncoderOffset;

    public SwerveModule(
        int driveMotorId, 
        int turningMotorId,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        double absoluteEncoderOffset
        )
    {

        this.absoluteEncoderOffset = absoluteEncoderOffset;
                
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId,MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);
        
        //int turningMotorResolution = 360/ (42 * gearRatio)

        absoluteEncoder = turningMotor.getAnalog(Mode.kAbsolute);
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MetersPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2MetersPerSec);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(50);
        turningMotor.setSmartCurrentLimit(30);

        driveMotor.burnFlash();
        turningMotor.burnFlash();
        
        //turningPidController = new ProfiledPIDController(.75, 0, 0, 
        //                                new TrapezoidProfile.Constraints(Math.PI, 2*Math.PI));
        turningPidController = new PIDController(.75, 0, 0); //.75, .01, 0 drivable
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

//        SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(, )

        resetEncoders();
    }

     public double getDrivePosition(){
        return driveEncoder.getPosition();
     }   

     public double getTurningPosition(){
      double angle = (absoluteEncoder.getPosition() * 310.303030303) + absoluteEncoderOffset;
      angle /= 1024;
      angle *= 360;
      if (angle >= 360) {
          return Math.toRadians (angle - 360);
      }
      else if (angle < 0) {
          return Math.toRadians(angle + 360);
      }
      else { 
          return Math.toRadians(angle);
      }
     }

     public double getDriveVelocity(){
        return driveEncoder.getVelocity();
     }
     
     public double getTurningVeloicty(){
        return turningEncoder.getVelocity();
     } 
     
     public double getAbsEncoderPos() {
        return (absoluteEncoder.getPosition() * 310.303030303) ;
    }
     public double getDriveAppOutput(){
        return driveMotor.getAppliedOutput();
     } 

     public double getSteerAppOutput(){
        return turningMotor.getAppliedOutput();
     }
     

    
  public double getAbsCurrentAngle() {
   double angle = (absoluteEncoder.getPosition() * 310.303030303) + absoluteEncoderOffset;
   angle /= 1024;
   angle *= 360;
   if (angle >= 360) {
       return (angle - 360);
   }
   else if (angle < 0) {
       return (angle + 360);
   }
   else { 
       return (angle);
   }
   //}
}
     
     public void resetEncoders(){
        driveEncoder.setPosition(0);
     }

 //    public void alignSteeting(){
 //       boolean alignSteering = true;
 //    }

     public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
     }

   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(getTurningPosition()));
  }
        
   public void setDesiredState(SwerveModuleState state, boolean zeroSwerves, boolean autoLevel, double gyroVal, boolean lockSwerves){
      if(zeroSwerves) {
         turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));
      }
      else if(autoLevel) {
         turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));
         if(gyroVal < -.25) {
            driveMotor.set(.1);
         }
         else if(gyroVal > .25) {
            driveMotor.set(-.1);
         }
         else {
            driveMotor.set(0);
         }
      }
      else { 
         if (Math.abs(state.speedMetersPerSecond) < 0.05) {
            stop();
            return;
         }
         state = SwerveModuleState.optimize(state, getState().angle);
         driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
         turningMotor.set((turningPidController.calculate(getTurningPosition(), state.angle.getRadians()))); 
      }

   }

     public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
     }

     public double getSetpoint() {
        return turningPidController.getSetpoint();
     }

     public CANSparkMax getDriveMotor() {
      return driveMotor;
     }
}
