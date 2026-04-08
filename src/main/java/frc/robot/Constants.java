// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (76 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static double MAX_SPEED  = Units.feetToMeters(14);

  public static final class VisionConstants {
    //Limelight Names
    public static final String FRONT_LIMELIGHT = "limelight"; 
    public static final String SIDE_LIMELIGHT = "side_limelight";

    //Distances
    public static final double shortDistance = -7.57;
    public static final double longDistance = -3.8;
    public static final double ALIGN_TX_OFFSET_DEG = 6.0;

    public static final double kP_Distance = 0.01;
    public static final double kP_Rotation = 0.4;
  }
  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }
  
  public static class ArmConstants{
    //CAN IDs
    public static final int leaderID = 14;
    public static final int followerID = 15;

    //Speeds
    public static final double upSpeed = 0.1;
    public static final double downSpeed = -0.075;

    //Stall Sequence Tuning Constants
    public static final double initialSpeed = -0.15;
    public static final double waitTime = 0.285;
    public static final double finalSpeed = 0.0005;

    //PID Constants
    public static final double kP= 0.0;
    public static final double kI= 0.0;
    public static final double kD = 0.0;

  }
  public static final class IntakeConstants {
    //CAN IDs
    public static final int leaderID = 20;     
    public static final int followerId = 21;

    public static final double SPEED = -0.35;
  }

    public static final class ShooterConstants{
      //Speeds
      public static final double conveyorRPM = -4000;
      public static final double rpm = -2750;
      public static final double indexSpeed = .25;
      public static final double runTime = 5;

      //CAN IDs
      public static final int conveyorID = 17;
      public static final int followConveyorID = 22;

      public static final int leadShooterID = 18;
      public static final int followShooterID = 19; 

      public static final int indexID = 23;

      //Tuning Constants
      public static final int kCurrentLimit = 80;
      public static final int kVoltageComp = 11;
      public static final int kToleranceRPM = 20;
      
      //Shooter PID
      public static final double kP= 0.001;
      public static final double kI= 0.0;
      public static final double kD = 0.0;
      public static final double kFF = 0.000167;

      //Conveyor PID
      public static final double kCP = 0.00001;
      public static final double kCI = 0;
      public static final double kCD= 0;
      public static final double kCFF = 0.000175;


    }
    

      
    
}

