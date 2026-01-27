package frc.robot.subsystems.swervedrive;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.signals.InvertedValue;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motorFx;
    private final VoltageOut m_voltReq;
    private final MotionMagicVoltage motionMagicRequest;
    private final DutyCycleOut dutyCycleRequest;
    private final SysIdRoutine sysIdRoutine;

    public IntakeSubsystem(){
        motorFx = new TalonFX(ArmConstants.ARM_MOTOR_CAN_ID);
        m_voltReq = new VoltageOut(0.0);
        configureMotor();
        sysIdRoutine = configSysId();
        motionMagicRequest = new MotionMagicVoltage(0);
        dutyCycleRequest = new DutyCycleOut(0);
    }
    
    private void configureMotor() {
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      config.Feedback.SensorToMechanismRatio = ArmConstants.TOTAL_GEAR_RATIO;
      
   
      config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.ARM_CRUISE_VELOCITY;
      config.MotionMagic.MotionMagicAcceleration = ArmConstants.ARM_ACCELERATION;
      config.MotionMagic.MotionMagicJerk = ArmConstants.ARM_JERK;
      
   
      config.Slot0.kP = ArmConstants.kP;
      config.Slot0.kI = ArmConstants.kI;
      config.Slot0.kD = ArmConstants.kD;
      
   
      config.Slot0.kS = ArmConstants.ARM_KS;
      config.Slot0.kG = ArmConstants.ARM_KG;
      config.Slot0.kV = ArmConstants.ARM_KV;
      config.Slot0.kA = ArmConstants.ARM_KA;
     
      config.Voltage.PeakForwardVoltage = ArmConstants.ARM_PEAK_FORWARD_VOLTAGE;
      config.Voltage.PeakReverseVoltage = ArmConstants.ARM_PEAK_REVERSE_VOLTAGE;
      
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = ArmConstants.ARM_MOTOR_INVERTED 
          ? InvertedValue.Clockwise_Positive 
          : InvertedValue.CounterClockwise_Positive;
      motorFx.getConfigurator().apply(config);
    }
    private SysIdRoutine configSysId(){
      return
          new SysIdRoutine(
         new SysIdRoutine.Config(
            null,    
            Volts.of(4), 
            null,       
            (state) -> SignalLogger.writeString("state", state.toString())
         ),
         new SysIdRoutine.Mechanism(
            (volts) -> motorFx.setControl(m_voltReq.withOutput(volts.in(Volts))),
            null,
            this
         )
      ); 
    }
    public void setPosition(double angle)
    {
      motorFx.setControl(motionMagicRequest.withPosition(angle));
    }
    public void setManualOutput(double percentOutput) {
      motorFx.setControl(dutyCycleRequest.withOutput(percentOutput));
  }
  public void stop()
  {
   motorFx.stopMotor();
  }
  public double getVelocity()
  {
   return motorFx.getVelocity().getValueAsDouble();
  }
  public double getPosition()
  {
   return motorFx.getPosition().getValueAsDouble();
  }

  public double getTargetDegrees()
  {
   return motorFx.getClosedLoopReference().getValueAsDouble();
  }

  public double getCurrentAmps() {
   return motorFx.getStatorCurrent().getValueAsDouble();
}
public double getVoltage() {
   return motorFx.getMotorVoltage().getValueAsDouble();
}
public SysIdRoutine getSysID(){

return this.sysIdRoutine;
}
}