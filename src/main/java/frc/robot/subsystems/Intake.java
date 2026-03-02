package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;

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
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;



public class Intake extends SubsystemBase{
   
   private SparkFlex leader = new SparkFlex(14, MotorType.kBrushless);
   private SparkFlex follower = new SparkFlex(15, MotorType.kBrushless);
   //private final TalonFX intakeMotor = new TalonFX(IntakeConstants.motorID);


   private SparkFlex intakeLead = new SparkFlex(20, MotorType.kBrushless);
   private SparkFlex intakeFollow = new SparkFlex(21, MotorType.kBrushless);

   SparkClosedLoopController m_controller = leader.getClosedLoopController();
   double setPoint = 0;
    private RelativeEncoder m_encoder = leader.getEncoder();
   
    private final SysIdRoutine sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null         // Use default timeout (10 s)
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> leader.setVoltage(volts.in(Volts)),
         null,
         this
      )
   );


    public void setIntakeSpeed(double speed) {
        intakeLead.set(speed);
    }
   

   public Intake(){
      

      configureMotors();
   }
   public void setPosition(double pos)
   {
      this.setPoint = pos;
   }


   public void configureMotors(){
      SparkFlexConfig leaderConfig = new SparkFlexConfig();
      SparkFlexConfig followerConfig = new SparkFlexConfig();
      followerConfig.follow(leader, true);


      SparkFlexConfig intakeLeadConfig = new SparkFlexConfig();
      SparkFlexConfig intakeFollowConfig = new SparkFlexConfig();

      intakeFollowConfig.follow(intakeLead, true);

      /*leaderConfig.closedLoop
      .p(0)
      .i(0)
      .d(0)
      .outputRange(0, 0);

      leaderConfig.closedLoop.feedForward
      .kS(0)
      .kV(0)
      .kA(0)
      .kG(0) // kG is a linear gravity feedforward, for an elevator
      .kCos(0) // kCos is a cosine gravity feedforward, for an arm
      .kCosRatio(0); // kCosRatio relates the encoder position to absolute position
      
      //leaderConfig.closedLoop.maxMotion
         .cruiseVelocity(0)
         .maxAcceleration(0)
         .allowedProfileError(0);
         /* /* */
      leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      intakeLead.configure(intakeLeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      intakeFollow.configure(intakeFollowConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}
   
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.quasistatic(direction);
   }

   public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.dynamic(direction);
   }
//   public void setSpeed(double speed)
//   {
//       intakeMotor.set(speed);
//   }

  public void setArmSpeed(double speed){
   leader.set(speed);
  }

  public void setPosition()
  {
      m_controller.setSetpoint(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      
  }
  public double getPosition()
  {
   return m_encoder.getPosition();
  }
}

