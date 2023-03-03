// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.MeasurementConstants.kFieldY;
import static frc.robot.Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.MeasurementConstants.kMaxSpeedMetersPerSecond;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase OneGPMobility(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("1GP", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.76, 4.92, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.14).withTimeout(1.5),
      new ArmToAngles(arm, 20.0, 150.0, true, 0.14).withTimeout(1.5),
      new ArmToAngles(arm, 36.0, 154.0, true, 0.14).withTimeout(1.5), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.3),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.3).withTimeout(1.5),
      new ArmToAngles(arm, 3.0, 0.0, true, 0.3).withTimeout(1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.76, kFieldY - 4.92, new Rotation2d(0.0)));
      }),
      new ParallelCommandGroup(
        new ArmToAngles(arm, 3.0, 0.0, true, 0.2),
        new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP)
      )
    );
  }

  public static CommandBase OneGPBalance(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", new InstantCommand(() -> {
      intake.CollectorOut(true, LEDs);
      intake.PivotIn(false);
      intake.SetCollector(0, 0.3);
    }).andThen(new WaitCommand(0.4)).andThen(new InstantCommand(() -> {
      intake.PivotIn(true);
    })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
      intake.SetCollector(0, 0.0);
    })));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false, LEDs);
      intake.SetCollector(0, 0.0);
    }));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("1GPBalanceCS", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.81, 2.2, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1.5),
      new ArmToAngles(arm, 20.0, 150.0, true, 0.15).withTimeout(1),
      new ArmToAngles(arm, 36.0, 154.0, true, 0.1).withTimeout(1.5), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.2).withTimeout(1),
      new ArmToAngles(arm, 3.0, 0.0, true, 0.2).withTimeout(1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.81, 5.8, new Rotation2d(0)));
      }),
      new ParallelCommandGroup(
        new ArmToAngles(arm, 3.0, 0.0, true, 0.2),
        new SequentialCommandGroup(
          new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
          new BalanceRobotOnChargingStation(drivetrain, () -> 0.3)
      ))
    );
  }

  public static CommandBase TwoGPCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new ParallelRaceGroup(new InstantCommand(() -> {
        intake.CollectorOut(true, LEDs);
        intake.PivotIn(false);
        intake.SetCollector(0, 0.3);
      }).andThen(new WaitCommand(0.9)).andThen(new InstantCommand(() -> {
        intake.PivotIn(true);
      })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
        intake.SetCollector(0, 0.0);
      })).andThen(new SequentialCommandGroup(
        new InstantCommand(() -> {
          intake.CollectorOut(false, LEDs);
        }),
        new WaitCommand(1.5))),
      new ArmToAngles(arm, 4.0, -12.5, false, 0.2))
      .andThen(new ParallelRaceGroup(
        new ArmToAngles(arm, 4.2, -13.5, false, 0.3),
          new SequentialCommandGroup(
          new WaitCommand(0.5),
          new InstantCommand(() -> arm.GrabGp(true)),
          new InstantCommand(() -> intake.PivotIn(false)),
          new WaitCommand(0.4),
          new WaitCommand(0.3))).andThen(
        new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(0.4))));
    
    // Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
    //   intake.CollectorOut(false, LEDs);
    //   intake.SetCollector(0, 0.0);
    // }));

    // Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
    //   new ArmToAngles(arm, 4.2, -11.2, false, 0.3),
    //   new WaitCommand(0.9),
    //   new InstantCommand(() -> arm.GrabGp(true)),
    //   new WaitCommand(0.2),
    //   new InstantCommand(() -> intake.PivotIn(false)),
    //   new WaitCommand(0.3),
    //   new ArmToAngles(arm, 4.0, 0.0, true, 0.2)));

    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("2GP", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.77, 4.96, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1),
      new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(0.6), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.4).withTimeout(0.3),
      new ArmToAngles(arm, 3.0, 0.0, true, 0.4).withTimeout(0.2),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.77, kFieldY - 4.96, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
      new AlignWithTag(drivetrain, 2).withTimeout(1.5),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1),
      new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 36.0, 154.0, true, 0.1).withTimeout(1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.2),
      new ArmToAngles(arm, 20.0, 154.0, false, 0.4).withTimeout(0.3),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.4).withTimeout(0.5));
  }   


  public static CommandBase TwoGP(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", new InstantCommand(() -> {
      intake.CollectorOut(true, LEDs);
      intake.PivotIn(false);
      intake.SetCollector(0, 0.3);
    }).andThen(new WaitCommand(0.4)).andThen(new InstantCommand(() -> {
      intake.PivotIn(true);
    })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
      intake.SetCollector(0, 0.0);
    })));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false, LEDs);
      intake.SetCollector(0, 0.0);
    }));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.2, -11.2, false, 0.3),
      new WaitCommand(0.7),
      new InstantCommand(() -> arm.GrabGp(true))));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("2GP", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence();
  }   


  // public static CommandBase ThreeGPBalanceNonCC(Drivetrain drivetrain, LEDs LEDs) {
  //   // Constants.AUTO_EVENT_MAP.put("Cube", new InstantCommand(() -> LEDs.setLEDS(Color.kPurple)));
  //   // Constants.AUTO_EVENT_MAP.put("Cone", new InstantCommand(() -> LEDs.setLEDS(Color.kYellow)));
  //   // Constants.AUTO_EVENT_MAP.put("UpdateOdometry", new InstantCommand(() -> drivetrain.updateOdometryIfTag()));
  //   List<PathPlannerTrajectory> AutoPaths;
  //   if (DriverStation.getAlliance() == Alliance.Red) {
  //     AutoPaths =
  //       PathPlanner.loadPathGroup(
  //         "R3GPBalanceNonCC",
  //         Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
  //         Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
  //   } else {
  //     AutoPaths =
  //       PathPlanner.loadPathGroup(
  //         "B3GPBalanceNonCC",
  //         Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
  //         Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
  //   }


  //   return Commands.sequence(
  //     // new InstantCommand(() -> drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d(Math.PI/2)))),
  //     new InstantCommand(() -> {
  //       if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d()));
  //       else drivetrain.setFieldPosition(new Pose2d( 1.81, 4.94, new Rotation2d(Math.PI/2)));
  //     }),
  //     // new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
  //     new FollowPathWithEvents(
  //       drivetrain.getCommandForTrajectory(AutoPaths.get(0)),
  //       AutoPaths.get(0).getMarkers(), 
  //       Constants.AUTO_EVENT_MAP),
  //     new AlignWithNode(drivetrain, 2).andThen(new AimAtNode(drivetrain)),
  //     // new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
  //     new FollowPathWithEvents(
  //       drivetrain.getCommandForTrajectory(AutoPaths.get(1)),
  //       AutoPaths.get(1).getMarkers(),
  //       Constants.AUTO_EVENT_MAP),
  //     new AlignWithNode(drivetrain, 1).andThen(new AimAtNode(drivetrain)),
  //     // new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
  //     new FollowPathWithEvents(
  //       drivetrain.getCommandForTrajectory(AutoPaths.get(2)),
  //       AutoPaths.get(2).getMarkers(),
  //       Constants.AUTO_EVENT_MAP)
  //   );
  // }

  // public static CommandBase TwoGPBalanceCC(Drivetrain drivetrain, LEDs LEDs) {
  //   // Constants.AUTO_EVENT_MAP.put("Cube", new InstantCommand(() -> LEDs.setLEDS(Color.kPurple)));
  //   // Constants.AUTO_EVENT_MAP.put("Cone", new InstantCommand(() -> LEDs.setLEDS(Color.kYellow)));
  //   // Constants.AUTO_EVENT_MAP.put("UpdateOdometry", new InstantCommand(() -> drivetrain.updateOdometryIfTag()));
  //   List<PathPlannerTrajectory> AutoPaths;
  //   if (DriverStation.getAlliance() == Alliance.Red) {
  //     AutoPaths =
  //       PathPlanner.loadPathGroup(
  //         "R2GPBalanceCC",
  //         Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
  //         Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
  //   } else {
  //     AutoPaths =
  //       PathPlanner.loadPathGroup(
  //         "B2GPBalanceCC",
  //         Constants.MeasurementConstants.kMaxSpeedMetersPerSecond,
  //         Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
  //   }

    
  //       return Commands.sequence(
  //         new InstantCommand(() -> {
  //           if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d()));
  //           else drivetrain.setFieldPosition(new Pose2d( 1.81, 4.94, new Rotation2d(Math.PI/2)));
  //         }),
  //         new FollowPathWithEvents(
  //           drivetrain.getCommandForTrajectory(AutoPaths.get(0)), 
  //           AutoPaths.get(0).getMarkers(), 
  //           Constants.AUTO_EVENT_MAP),
  //           new AlignWithNode(drivetrain, 2).andThen(new AimAtNode(drivetrain)),
  //           // new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
  //           new FollowPathWithEvents(
  //             drivetrain.getCommandForTrajectory(AutoPaths.get(1)),
  //             AutoPaths.get(1).getMarkers(),
  //             Constants.AUTO_EVENT_MAP)

  //       );
  //   }
}
