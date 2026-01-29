// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(29.5)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static double MAX_SPEED  = Units.feetToMeters(14);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }


    public static final class VisionConstants {

        // =======================================================
        // APRILTAG FOLLOWING PARAMETERS
        // =======================================================

        /** Tag ID to follow */
        public static final int FOLLOW_TAG_ID = 1;

        /** Desired distance from the AprilTag (in meters) */
        public static final double FOLLOW_TAG_DISTANCE_METERS = 1.00;

        /** Forward/back proportional gain */
        public static final double kDistanceP = 0.90;

        /** Rotational proportional gain */
        public static final double kYawP = 0.020;


        // =======================================================
        // CAMERA MOUNTING PARAMETERS (ROBOT → CAMERA TRANSFORM)
        // Replace these with YOUR robot’s actual measurements.
        // =======================================================

        /** Camera mounting translation from robot center */
        public static final Translation3d CAMERA_TRANSLATION =
                new Translation3d(
                        Units.inchesToMeters(12.0),   // forward from center
                        Units.inchesToMeters(0.0),    // sideways (positive = left)
                        Units.inchesToMeters(10.0));  // height

        /** Camera mounting rotation */
        public static final Rotation3d CAMERA_ROTATION =
                new Rotation3d(
                        0.0,                              // roll
                        Units.degreesToRadians(-20),       // pitch down
                        0.0);                              // yaw
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

  public static class Motors{
    public static int leftMotor = 13;
    public static int rightMotor = 14;  }
  
  public static class ArmConstants{

    public static final double positionUp = 0; //Set based on set positons
    public static final double positionDown = 0; // Set based on set positions
    public static final int ARM_MOTOR_CAN_ID = 10; // Change to CAN ID
    public static final int ARM_MOTOR2_CAN_ID = 11;
    public static final boolean ARM_MOTOR_INVERTED = false; // Set based on testing
    //PID Constants
    public static final double kP= 0.0;
    public static final double kI= 0.0;
    public static final double kD = 0.0;
    // Gear ratios
    public static final double CYCLOIDAL_GEAR_RATIO = 81.0; // Your cycloidal ratio (43, 81, 100, 121, etc.)
    public static final double ADDITIONAL_REDUCTION = 1.0; // Any extra gearing (1.0 if none)
    public static final double TOTAL_GEAR_RATIO = CYCLOIDAL_GEAR_RATIO * ADDITIONAL_REDUCTION;

// Physical measurements
    public static final double ARM_LENGTH_METERS = 0.5; // Distance from pivot to end of arm
    public static final double ARM_MASS_KG = 3.0; // Total arm mass
    public static final double ARM_CENTER_OF_MASS_METERS = 0.25; // Distance from pivot to CoM
    // Feedforward gains (from characterization)
    public static final double ARM_KS = 0.0; // Static friction (Volts) - TUNE MANUALLY or SysId
    public static final double ARM_KG = 0.0; // Gravity (Volts) - CRITICAL! Tune this first
    public static final double ARM_KV = 0.0; // Velocity (Volts per deg/s) - From SysId
    public static final double ARM_KA = 0.0; // Acceleration (Volts per deg/s²) - From SysId
    public static final double ARM_PEAK_FORWARD_VOLTAGE = 12.0;
    public static final double ARM_PEAK_REVERSE_VOLTAGE = -12.0;
    
    // ===== MOTION MAGIC =====
    public static final double ARM_CRUISE_VELOCITY = 60.0;
    public static final double ARM_ACCELERATION = 120.0;
    public static final double ARM_JERK = 1200.0;

    // In ArmConstants class, add this section:

    // ===== SYSID CONFIGURATION =====
    public static final double SYSID_RAMP_RATE = 1.0; // Volts per second
    public static final double SYSID_STEP_VOLTAGE = 7.0; // Volts for dynamic test
    public static final double SYSID_TIMEOUT = 10.0; // Seconds max per test

    // POSITION
    public static final double INTAKE_UP_POSITION = 0.0;
    public static final double INTAKE_DOWN_POSITION = 0.25;
    public static final double ARM_POSITION_TOLERANCE = 0.01;

  }


  }