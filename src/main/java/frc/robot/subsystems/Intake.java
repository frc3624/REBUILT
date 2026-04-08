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
import com.revrobotics.spark.config.SparkBaseConfig;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;



public class Intake extends SubsystemBase{
   

   private SparkFlex leader = new SparkFlex(ArmConstants.leaderID, MotorType.kBrushless);
   private SparkFlex follower = new SparkFlex(ArmConstants.followerID, MotorType.kBrushless);
   //private final TalonFX intakeMotor = new TalonFX(IntakeConstants.motorID);
   private final double kP = 3;
   private final double kI = 0;
   private final double kD = 0;
   private final double kG = .5;
   private final double kS = 0;
   private final double kV = 0;
   private static final double maxVelocity = 100;    // degrees per second
   private static final double maxAcceleration = 50; // degrees per second squared
   private static final double allowedError = 1.0;  // degrees
   private double targetDegrees = -30;
   private SparkFlex intakeLead = new SparkFlex(IntakeConstants.leaderID, MotorType.kBrushless);
   private SparkFlex intakeFollow = new SparkFlex(IntakeConstants.followerId, MotorType.kBrushless);


   private final ProfiledPIDController m_pid = new ProfiledPIDController(
    kP, kI, kD,
    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration)
);

private final ArmFeedforward m_ff = new ArmFeedforward(kS, kG, kV);

    
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

      intakeLeadConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
      followerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);

      
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

      // double pidOutput = m_pid.calculate(getPosition(), targetDegrees);
      
      // // ff uses the PROFILE's current angle setpoint, not the final target
      // double ffOutput = m_ff.calculate(
      //     Math.toRadians(m_pid.getSetpoint().position),
      //     Math.toRadians(m_pid.getSetpoint().velocity)
      // );

      // leader.setVoltage(pidOutput + ffOutput);

    
    SmartDashboard.putNumber("Intake degrees", this.targetDegrees);
                            
    //SmartDashboard.putNumber("CurrentPose", getPosition());
  }
  
}

