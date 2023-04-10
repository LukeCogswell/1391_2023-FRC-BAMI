// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.MeasurementConstants.kInchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kRotationUnits = "degrees";
  public static final String kDistanceUnits = "meters";

  public static final class OIConstants {
    public static final int kDriverControllerID = 0;
    public static final int kAButtonID = 1;
    public static final int kBButtonID = 2;
    public static final int kXButtonID = 3;
    public static final int kYButtonID = 4;
    public static int kDriverControllerPort = 0;
    public static int kOperatorControllerPort = 1;

  }

  public static final class MeasurementConstants {
    // This is based on the CAD model (divided by two to represent distance from center of robot) 
    public static final double kInchesToMeters = 39.37;
    public static final double kModuleXOffsetMeters = 21.5 / kInchesToMeters / 2; // 21.5 inches - distance between left and right wheels
    public static final double kModuleYOffsetMeters = 18.5 / kInchesToMeters / 2; // 18.5 inches - distance between front and back wheels
    public static final double kWheelDiameterMeters = 0.10033; // 4 inches - diameter of the wheels

    public static final double kCameraOffsetX = 15 / kInchesToMeters;
    public static final double kCameraOffsetY = 0;
    public static final double kCameraOffsetZ = 15 / kInchesToMeters;

    public static final double kHybridNodeDepth = 16 / kInchesToMeters;
    public static final double kNodeOffset = 20.25 / kInchesToMeters;

    public static final double kFieldX = 649 / kInchesToMeters;
    public static final double kFieldY = 319 / kInchesToMeters;

    public static final double kFrontLeftEncoderOffset = 88.77;//86.1     11
    public static final double kBackLeftEncoderOffset = 177.13;//179          12
     public static final double kFrontRightEncoderOffset = 264.79;// 269.12  10
    public static final double kBackRightEncoderOffset = 22.06;// 27.8       9 

    public static final double kShoulderEncoderOffset = 348.9; // FIND THIS
    public static final double kElbowEncoderOffset = 14.05; // FIND THIS
    
    public static final double kShoulderLength = 42;
    public static final double kElbowLength = 34;

    public static final double kShoulderLengthSquared = kShoulderLength * kShoulderLength;
    public static final double kElbowLengthSquared = kElbowLength * kElbowLength;

    public static final double kShoulderMaxSpeed = 0.25;
    public static final double kElbowMaxSpeed = 0.5; 

    public static final double kMaxReach = 56; //inches
    public static final double kMaxHeight = 54;

    public static final double kMaxSpeedMetersPerSecond = 5880 / 60.0 *
      SwerveModuleConstants.kDriveReduction *
      MeasurementConstants.kWheelDiameterMeters * Math.PI; // ~ 4.6 m/s
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
      Math.hypot(kModuleXOffsetMeters / 2.0, kModuleYOffsetMeters / 2.0);
  }
  
  public static final class BalancingConstants {
    public static final double kAngleTolerance = 10.0; //degrees
    public static final double kTriggerMultiplier = 0.2; 
    public static final double kSpeedLimit = 0.45; 
  }

  public static final class SwerveModuleConstants {
    public static final double kSpeedMultiplier = 1; // limits robot speed
    public static final double kRotationSpeedMultiplier = 0.7;
    public static final double kDriveDeadband = 0.05;

    public static final double kMaxVoltage = 12.0;
    public static final double kAccelerationSeconds = 0.5; // 0.5 seconds to reach full speed

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kSteerMotorCurrentLimit = 20;

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Wheel revolutions per motor revolution`
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // Module revolutions per motor revolution

    public static final double kDriveEncoderPositionConversionFactor = Math.PI * MeasurementConstants.kWheelDiameterMeters * kDriveReduction;
    public static final double kSteerEncoderPositionConversionFactor = 360 * kSteerReduction; 

    public static final class PID {
      public static final double kSteerP = 0.01;
      public static final double kSteerI = 0.0; // Used in module control
      public static final double kSteerD = 0.0;

      public static final double kModDriveP = 0.18;
      public static final double kModDriveI = 0.0; // Used in module control TODO: Tune post-competition
      public static final double kModDriveD = 0.0;

      public static final double kDriveP = 5.0;
      public static final double kDriveI = 0.0; // Used in pose control
      public static final double kDriveD = 0.0;

      public static final double kTurnP = 1.15;
      public static final double kTurnI = 0.0; // Used in pose control
      public static final double kTurnD = 0.0;

      public static final double kTiltP = 0.01;
      public static final double kTiltI = 0.0;
      public static final double kTiltD = 0.0;

      public static final double kDriveTolerance = 0.01;
      public static final double kTurnTolerance = 1.0;
    }
    
  }

  public static final class ArmConstants {
    public static final double kShoulderMaxAngle = 45; //degrees
    public static final double kElbowMaxAngle = 130; //degrees

    public static final class PID {
      public static final double kShoulderP = 0.016;
      public static final double kShoulderI = 0.016;
      public static final double kShoulderD = 0.00001;

      public static final double kElbowP = 0.014;
      public static final double kElbowI = 0.0015;
      public static final double kElbowD = 0.000;// 0.0005 had oscillations
    }
  }


  public static final class CANConstants {
    public static final int kFrontLeftDriveMotorID = 3;
    public static final int kBackLeftDriveMotorID = 2;
    public static final int kFrontRightDriveMotorID = 4;
    public static final int kBackRightDriveMotorID = 1;

    public static final int kFrontLeftSteerMotorID = 7;
    public static final int kBackLeftSteerMotorID = 6;
    public static final int kFrontRightSteerMotorID = 8;
    public static final int kBackRightSteerMotorID = 5;

    public static final int kFrontLeftEncoderID = 11; //9
    public static final int kBackLeftEncoderID = 12;// 12
    public static final int kFrontRightEncoderID = 10;// 11
    public static final int kBackRightEncoderID = 9;// 10

    public static final int kElbowEncoderID = 14;
    public static final int kShoulderEncoderID = 13;

    public static final int kLeftShoulderMotorID = 17;
    public static final int kRightShoulderMotorID = 18;
    public static final int kLeftElbowMotorID = 19;
    public static final int kRightElbowMotorID = 20;

    public static final int kLeftCollectorMotorID = 16;
    public static final int kRightCollectorMotorID = 15;


    public static final double kEncoderResolution = 4096;
    public static final double kEncoderDistancePerPulse =
        (MeasurementConstants.kWheelDiameterMeters * Math.PI) / kEncoderResolution;
  }

  public static HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

public static final class AprilTagFieldLayouts {
  private static final double kGridTagHeight = 18.22 / kInchesToMeters;
  private static final double kSubstationTagHeight = 27.38 / kInchesToMeters;
  private static final double kBlueTagX = 40.45 / kInchesToMeters;
  // private static final double kRedTagX = 610.77 / kInchesToMeters;
  // public static final AprilTag TagId1 = new AprilTag(1, new Pose3d(kRedTagX, 42.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 180)));
  // public static final AprilTag TagId2 = new AprilTag(2, new Pose3d(kRedTagX, 108.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 180)));
  // public static final AprilTag TagId3 = new AprilTag(3, new Pose3d(kRedTagX, 174.19/ kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 180)));
  public static final AprilTag TagId1 = new AprilTag(1, new Pose3d(kBlueTagX, 174.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId2 = new AprilTag(2, new Pose3d(kBlueTagX, 108.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId3 = new AprilTag(3, new Pose3d(kBlueTagX, 42.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId4 = new AprilTag(4, new Pose3d(14.25 / kInchesToMeters, -49.36 / kInchesToMeters, kSubstationTagHeight, new Rotation3d(0, 0, 180)));
  public static final AprilTag TagId5 = new AprilTag(5, new Pose3d(14.25 / kInchesToMeters, 265.74 / kInchesToMeters, kSubstationTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId6 = new AprilTag(6, new Pose3d(kBlueTagX, 174.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId7 = new AprilTag(7, new Pose3d(kBlueTagX, 108.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final AprilTag TagId8 = new AprilTag(8, new Pose3d(kBlueTagX, 42.19 / kInchesToMeters, kGridTagHeight, new Rotation3d(0, 0, 0)));
  public static final List<AprilTag> AprilTagList = Arrays.asList(TagId1, TagId2, TagId3, TagId4, TagId5, TagId6, TagId7, TagId8);
  public static final AprilTagFieldLayout AprilTagFullFieldLayout = new AprilTagFieldLayout(AprilTagList, 50.0, 50.0);

}

}
