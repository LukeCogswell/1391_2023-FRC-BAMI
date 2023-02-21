// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import static frc.robot.Constants.MeasurementConstants.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase OneGPBalance(Drivetrain drivetrain, Arm arm, Intake intake) {
    Constants.AUTO_EVENT_MAP.put("Collect", new InstantCommand(() -> {
      intake.CollectorOut(true);
      intake.SetCollector(0, 0.3);
    }).andThen(new WaitCommand(0.4)).andThen(new InstantCommand(() -> {
      intake.PivotIn(true);
    })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
      intake.SetCollector(0, 0.0);
    })));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("1GPBalanceCS", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.81, 2.2, new Rotation2d(-drivetrain.getNavxYaw())))),
      // new ArmToAngles(arm, -8.0, 90.0, true, 0.5).withTimeout(1),
      // new ArmToAngles(arm, 20.0, 120.0, true, 0.2).withTimeout(1),
      // new ArmToAngles(arm, 36.0, 154.0,, true, 0.1).withTimeout(1), //Score High
      // new InstantCommand(() -> arm.GrabGp(false)),
      // new WaitCommand(0.5),
      // new ArmToAngles(arm, -8.0, 90.0, false, 0.5).withTimeout(1),
      // new ArmToAngles(arm, 3.0, 0.0, true, 0.5),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.81, 5.8, new Rotation2d(-drivetrain.getNavxYaw())));
      }),
      new ParallelCommandGroup(
        new ArmToAngles(arm, 3.0, 0.0, true, 0.2),
        new SequentialCommandGroup(
          new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
          new BalanceRobotOnChargingStation(drivetrain, () -> 0.2)
      ))
    );
  }


  public static CommandBase ThreeGPBalanceNonCC(Drivetrain drivetrain, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Cube", new InstantCommand(() -> LEDs.setLEDS(Color.kPurple)));
    Constants.AUTO_EVENT_MAP.put("Cone", new InstantCommand(() -> LEDs.setLEDS(Color.kYellow)));
    // Constants.AUTO_EVENT_MAP.put("UpdateOdometry", new InstantCommand(() -> drivetrain.updateOdometryIfTag()));
    List<PathPlannerTrajectory> AutoPaths;
    if (DriverStation.getAlliance() == Alliance.Red) {
      AutoPaths =
        PathPlanner.loadPathGroup(
          "R3GPBalanceNonCC",
          Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
          Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
    } else {
      AutoPaths =
        PathPlanner.loadPathGroup(
          "B3GPBalanceNonCC",
          Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
          Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
    }


    return Commands.sequence(
      // new InstantCommand(() -> drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d(Math.PI/2)))),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d()));
        else drivetrain.setFieldPosition(new Pose2d( 1.81, 4.94, new Rotation2d(Math.PI/2)));
      }),
      new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
      new FollowPathWithEvents(
        drivetrain.getCommandForTrajectory(AutoPaths.get(0)),
        AutoPaths.get(0).getMarkers(), 
        Constants.AUTO_EVENT_MAP),
      new AlignWithNode(drivetrain, 2).andThen(new AimAtNode(drivetrain)),
      new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
      new FollowPathWithEvents(
        drivetrain.getCommandForTrajectory(AutoPaths.get(1)),
        AutoPaths.get(1).getMarkers(),
        Constants.AUTO_EVENT_MAP),
      new AlignWithNode(drivetrain, 1).andThen(new AimAtNode(drivetrain)),
      new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
      new FollowPathWithEvents(
        drivetrain.getCommandForTrajectory(AutoPaths.get(2)),
        AutoPaths.get(2).getMarkers(),
        Constants.AUTO_EVENT_MAP)
    );
  }

  public static CommandBase TwoGPBalanceCC(Drivetrain drivetrain, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Cube", new InstantCommand(() -> LEDs.setLEDS(Color.kPurple)));
    Constants.AUTO_EVENT_MAP.put("Cone", new InstantCommand(() -> LEDs.setLEDS(Color.kYellow)));
    // Constants.AUTO_EVENT_MAP.put("UpdateOdometry", new InstantCommand(() -> drivetrain.updateOdometryIfTag()));
    List<PathPlannerTrajectory> AutoPaths;
    if (DriverStation.getAlliance() == Alliance.Red) {
      AutoPaths =
        PathPlanner.loadPathGroup(
          "R2GPBalanceCC",
          Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
          Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
    } else {
      AutoPaths =
        PathPlanner.loadPathGroup(
          "B2GPBalanceCC",
          Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
          Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    
        return Commands.sequence(
          new InstantCommand(() -> {
            if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d()));
            else drivetrain.setFieldPosition(new Pose2d( 1.81, 4.94, new Rotation2d(Math.PI/2)));
          }),
          new FollowPathWithEvents(
            drivetrain.getCommandForTrajectory(AutoPaths.get(0)), 
            AutoPaths.get(0).getMarkers(), 
            Constants.AUTO_EVENT_MAP),
            new AlignWithNode(drivetrain, 2).andThen(new AimAtNode(drivetrain)),
            new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
            new FollowPathWithEvents(
              drivetrain.getCommandForTrajectory(AutoPaths.get(1)),
              AutoPaths.get(1).getMarkers(),
              Constants.AUTO_EVENT_MAP)

        );
    }
}
