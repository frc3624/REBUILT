package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;



public class Intake extends SubsystemBase{
   

   private SparkFlex leader = new SparkFlex(14, MotorType.kBrushless);
   private SparkFlex follower = new SparkFlex(15, MotorType.kBrushless);
   //private final TalonFX intakeMotor = new TalonFX(IntakeConstants.motorID);
   private final double kP = 3;
   private final double kI = 0;
   private final double kD = 0;
   private final double kG = .5;
private static final double maxVelocity = 100;    // degrees per second
private static final double maxAcceleration = 50; // degrees per second squared
private static final double allowedError = 1.0;  // degrees
   private double targetDegrees = -30;
   private SparkFlex intakeLead = new SparkFlex(20, MotorType.kBrushless);
   private SparkFlex intakeFollow = new SparkFlex(21, MotorType.kBrushless);

    
    private static final double GEAR_RATIO = 15.0;

   SparkClosedLoopController m_controller = leader.getClosedLoopController();
   double setPoint = 0;
    private RelativeEncoder m_encoder = leader.getEncoder();
   //private static final double kG = 0.3;

    public void setIntakeSpeed(double speed) {
        intakeLead.set(speed);
    }
   

   public Intake(){
      

      configureMotors();
   }
   /*public void setPosition(double pos)
   {
      this.setPoint = pos;
      double ff = kG * Math.cos(Units.rotationsToRadians(m_encoder.getPosition()));
      m_controller.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);

   }*/

   

   public void configureMotors(){
      SparkFlexConfig leaderConfig = new SparkFlexConfig();
      SparkFlexConfig followerConfig = new SparkFlexConfig();
      followerConfig.follow(leader, true);
      leaderConfig.encoder
    .positionConversionFactor(360.0 / GEAR_RATIO)        // degrees
    .velocityConversionFactor(360.0 / GEAR_RATIO / 60.0);
      leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .maxMotion
                .cruiseVelocity(maxVelocity)
                .maxAcceleration(maxAcceleration)
                .allowedProfileError(allowedError);
                


      SparkFlexConfig intakeLeadConfig = new SparkFlexConfig();
      SparkFlexConfig intakeFollowConfig = new SparkFlexConfig();

      intakeFollowConfig.follow(intakeLead, true);

      
      leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      intakeLead.configure(intakeLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intakeFollow.configure(intakeFollowConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}
   
//   public void setSpeed(double speed)
//   {
//       intakeMotor.set(speed);
//   }

  public void setArmSpeed(double speed){
   leader.set(speed);
  }
public boolean reached(){
    if((targetDegrees - 0.003 < getPosition()) && (getPosition() < targetDegrees + 0.003)){
      return true;
    }
    return false;
  }

    public void setPosition(double degrees) {
      targetDegrees = degrees;
      
    }


  public double getPosition()
  {
   return m_encoder.getPosition();
  }
  
  public void runFeedFoward(){
   double ff = kG * Math.cos(Math.toRadians(m_encoder.getPosition()));
    leader.setVoltage(ff);
  }

  @Override 
  public void periodic(){
   /*double ff = kG * Math.cos(Math.toRadians(m_encoder.getPosition()));
    leader.setVoltage(ff);*/
   
   SmartDashboard.putNumber("Intake degrees", this.targetDegrees);
                           
  SmartDashboard.putNumber("CurrentPose", getPosition());
                           }
  
}

