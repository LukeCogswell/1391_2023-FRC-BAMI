// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.MeasurementConstants.kFieldY;
import static frc.robot.Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.MeasurementConstants.kMaxSpeedMetersPerSecond;

import java.util.List;

import javax.sound.midi.Sequencer;

import org.w3c.dom.Node;

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
      intake.CollectorOut(true);
      intake.PivotIn(false);
    }).andThen(new WaitCommand(0.4)).andThen(new InstantCommand(() -> {
      intake.SetCollector(0, 0.3);
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
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.81, 2.2, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1.5),
      new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(1),
      new ArmToAngles(arm, 36.0, 155.0, true, 0.1).withTimeout(1.5), //Score High
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
          new BalanceRobotOnChargingStation(drivetrain, () -> 0.4)
      ))
    );
  }

  public static CommandBase TwoGPCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new ParallelRaceGroup(new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }).andThen(new WaitCommand(0.9)).andThen(new InstantCommand(() -> {
        intake.SetCollector(0, 0.3);
        intake.PivotIn(true);
      })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
        intake.SetCollector(0, 0.0);
      })).andThen(new SequentialCommandGroup(
        new InstantCommand(() -> {
          intake.CollectorOut(false);
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
    //   intake.CollectorOut(false);
    //   intake.SetCollector(0, 0.0);
    // }));
    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(1.2),
      new WaitCommand(0.9),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.2),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.3),
      new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(2)));

    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    List<PathPlannerTrajectory> AutoPaths = 
      PathPlanner.loadPathGroup("2.5GPCC", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.79, 0.53, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.5), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.77, kFieldY - 0.53, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(0)), AutoPaths.get(0).getMarkers(), Constants.AUTO_EVENT_MAP),
      new AlignOnTag(drivetrain, 2).withTimeout(1.8),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(0.7),
      // new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      // new ParallelCommandGroup(new ArmToAngles(arm, 4.0, 0.0, false, null),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(1)), AutoPaths.get(1).getMarkers(), Constants.AUTO_EVENT_MAP));
  }   
 
  public static CommandBase TwoPlusOneGPCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new ParallelRaceGroup(new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }).andThen(new WaitCommand(0.9)).andThen(new InstantCommand(() -> {
        intake.SetCollector(0, 0.3);
        intake.PivotIn(true);
      })).andThen(new WaitCommand(0.7)).andThen(new InstantCommand(() -> {
        intake.SetCollector(0, 0.0);
      })).andThen(new SequentialCommandGroup(
        new InstantCommand(() -> {
          intake.CollectorOut(false);
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
    //   intake.CollectorOut(false);
    //   intake.SetCollector(0, 0.0);
    // }));
    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(1.2),
      new WaitCommand(0.9),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.2),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.3),
      new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(2)));

    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("2GPCC", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.79, 0.53, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.5), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.77, kFieldY - 0.53, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
      new AlignWithTag(drivetrain, 2).withTimeout(1.8),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(0.7),
      // new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.2),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP));
  }   


  public static CommandBase TwoGP(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    
    
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }),
      new WaitCommand(0.9),
      new InstantCommand(() -> {
        intake.SetCollector(0, 0.3);
        intake.PivotIn(true);
      }),
      new WaitCommand(0.7),
      new InstantCommand(() -> intake.SetCollector(0, 0.0))));
      //   new WaitCommand(1.5))),
      // new ArmToAngles(arm, 4.0, -12.5, false, 0.2))
      // .andThen(new ParallelRaceGroup(
      //   new ArmToAngles(arm, 4.2, -13.5, false, 0.3),
      //     new SequentialCommandGroup(
      //     new WaitCommand(0.5),
      //     new InstantCommand(() -> arm.GrabGp(true)),
      //     new InstantCommand(() -> intake.PivotIn(false)),
      //     new WaitCommand(0.4),
      //     new WaitCommand(0.3))).andThen(
      //   new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(0.4))));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.8),
      new WaitCommand(0.1),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(2)));

    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    PathPlannerTrajectory AutoPath = 
      PathPlanner.loadPath("2GP", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.76, 4.96, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.2), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.4).withTimeout(0.4),
      new ArmToAngles(arm, 3.0, 0.0, true, 0.4).withTimeout(0.4),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.76, kFieldY - 4.96, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
      new ParallelCommandGroup(new AlignWithTag(drivetrain, 2).withTimeout(1.4),
      new InstantCommand(() -> arm.GrabGp(true)).andThen(new WaitCommand(0.1).andThen(new InstantCommand(() -> intake.PivotIn(false))))),
      // new AlignWithTag(drivetrain, 2).withTimeout(1.4),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(1),
      // new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.2),
      new ArmToAngles(arm, 20.0, 154.0, false, 0.4).withTimeout(0.3),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.4).withTimeout(0.5),
      new ArmToAngles(arm, -4.0, 0.0, false, null));
  }   


  public static CommandBase TwoPlusOneGP(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    
    
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }),
      new WaitCommand(0.9),
      new InstantCommand(() -> {
        intake.SetCollector(0, 0.3);
        intake.PivotIn(true);
      }),
      new WaitCommand(0.7),
      new InstantCommand(() -> intake.SetCollector(0, 0.0))));
      //   new WaitCommand(1.5))),
      // new ArmToAngles(arm, 4.0, -12.5, false, 0.2))
      // .andThen(new ParallelRaceGroup(
      //   new ArmToAngles(arm, 4.2, -13.5, false, 0.3),
      //     new SequentialCommandGroup(
      //     new WaitCommand(0.5),
      //     new InstantCommand(() -> arm.GrabGp(true)),
      //     new InstantCommand(() -> intake.PivotIn(false)),
      //     new WaitCommand(0.4),
      //     new WaitCommand(0.3))).andThen(
      //   new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(0.4))));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.8),
      new WaitCommand(0.1),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, 4.0, 0.0, true, 0.2).withTimeout(2)));

    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    List<PathPlannerTrajectory> AutoPaths = 
      PathPlanner.loadPathGroup("2.5GP", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.76, 4.96, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.2), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -8.0, 90.0, false, 0.4).withTimeout(0.4),
      new ArmToAngles(arm, 3.0, 0.0, true, 0.4).withTimeout(0.4),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.76, kFieldY - 4.96, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(0)), AutoPaths.get(0).getMarkers(), Constants.AUTO_EVENT_MAP),
      new ParallelCommandGroup(new AlignOnTag(drivetrain, 2).withTimeout(1.4),
      new InstantCommand(() -> arm.GrabGp(true)).andThen(new WaitCommand(0.1).andThen(new InstantCommand(() -> intake.PivotIn(false))))),
      // new AlignWithTag(drivetrain, 2).withTimeout(1.4),
      new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new ParallelRaceGroup(
        new ArmToAngles(arm, 4.0, 0.0, false, null),
        new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(1)), AutoPaths.get(1).getMarkers(), Constants.AUTO_EVENT_MAP)),
      new WaitCommand(0.2),
      new InstantCommand(() -> {
        intake.CollectorOut(false);
        intake.SetCollector(0, 0.0);
      }));

      
  }   
  
  public static CommandBase TwoGPBalanceCS(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Collect", new SequentialCommandGroup(
    new InstantCommand(() -> {
      intake.CollectorOut(true);
      intake.PivotIn(false);
    }).andThen(new WaitCommand(0.8)).andThen(new InstantCommand(() -> {
      intake.PivotIn(true);
    })),
    new WaitCommand(0.3),
    new InstantCommand(() ->  intake.SetCollector(0, 0.3)),
    new WaitCommand(0.4),
    new InstantCommand(() ->  intake.SetCollector(0, 0.0))
    ));

    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
    }));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(2),
      new InstantCommand(() -> arm.GrabGp(true)),
      // new WaitCommand(0.2),
      new InstantCommand(() -> intake.PivotIn(false))));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));


    PathPlannerTrajectory AutoPath = 
    PathPlanner.loadPath("2GPBalanceCS", 2.0, kMaxAccelerationMetersPerSecondSquared);
    
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d(1.77, 2.18, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.7),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.1), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.77, kFieldY - 2.18, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPath), AutoPath.getMarkers(), Constants.AUTO_EVENT_MAP),
      new ParallelCommandGroup(
        new AlignWithTag(drivetrain, 2).withTimeout(1.5),
        new SequentialCommandGroup(
          new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(0.5),
          new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(1), //Score High
          new InstantCommand(() -> arm.GrabGp(false))//,
          // new WaitCommand(0.1)
          )),
          new ParallelCommandGroup(
            new ArmToAngles(arm, 4.0, 0.0, false, null),
            new SequentialCommandGroup(
              new ChargeWithOdometry(drivetrain, 4.6), //4.45
              new BalanceRobotOnChargingStation(drivetrain, () -> 0.4))
              )
              
              );
            }
           
     
  public static CommandBase ThreeGPNonCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }),
      new WaitCommand(0.9),
      new InstantCommand(() -> {
        intake.SetCollector(0, 0.3);
        intake.PivotIn(true);
      }),
      new WaitCommand(0.7),
      new InstantCommand(() -> intake.SetCollector(0, 0.0))));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.8),
      new WaitCommand(0.1),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, 4.0, 100.0, true, 0.2).withTimeout(2)));


    Constants.AUTO_EVENT_MAP.put("ScoreCubeLow", new SequentialCommandGroup(
      new InstantCommand(() -> intake.CollectorOut(true)),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake.SetCollector(0, -0.4)),
      new WaitCommand(0.6),
      new InstantCommand(() -> {
        intake.SetCollector(0, 0.0);
        intake.CollectorOut(false);
      })
    ));
    
    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    List<PathPlannerTrajectory> AutoPaths = 
      PathPlanner.loadPathGroup("3GPNonCCHL", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.76, 4.96, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.2), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.76, kFieldY - 4.96, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(0)), AutoPaths.get(0).getMarkers(), Constants.AUTO_EVENT_MAP),
      new SequentialCommandGroup(new InstantCommand(() -> arm.GrabGp(true)).andThen(new WaitCommand(0.1).andThen(new InstantCommand(() -> intake.PivotIn(false)))),
      // new AlignWithTag(drivetrain, 2).withTimeout(1.4),
      // new ArmToAngles(arm, -8.0, 90.0, true, 0.15).withTimeout(0.3),
      // new ArmToAngles(arm, 20.0, 154.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(0.8), //Score High
      new InstantCommand(() -> arm.GrabGp(false))),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(1)), AutoPaths.get(1).getMarkers(), Constants.AUTO_EVENT_MAP),
      new WaitCommand(0.2),
      new InstantCommand(() -> {
        intake.CollectorOut(false);
        intake.SetCollector(0, 0.0);
      }));
      // new DriveForDistanceInDirection(drivetrain, 4.0, 0.0));

      
  }   


  public static CommandBase LinkHighNonCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    

    Constants.AUTO_EVENT_MAP.put("Collect", 
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }),
      new WaitCommand(0.9),
      new InstantCommand(() -> {
        intake.PivotIn(true);
      }),
      new WaitCommand(0.3),
      new InstantCommand(() ->intake.SetCollector(0, 0.3)),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake.SetCollector(0, 0.0))));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    // Constants.AUTO_EVENT_MAP.put("ExtendArm", new ArmToAngles(arm, 4.0, 90.0, true, null));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.92),
      // new WaitCommand(0.1),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -4.0, 100.0, true, 0.2)));
      
      Constants.AUTO_EVENT_MAP.put("TransferCone", new SequentialCommandGroup(
        new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.83),
        // new WaitCommand(0.1),
        new InstantCommand(() -> arm.GrabGp(true)),
        new WaitCommand(0.1),
        new InstantCommand(() -> intake.PivotIn(false)),
        new WaitCommand(0.1),
        new ArmToAngles(arm, -8.0, 110.0, true, 0.2)));
    
    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    List<PathPlannerTrajectory> AutoPaths = 
      PathPlanner.loadPathGroup("HighLinkNonCC", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.76, 4.96, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.0), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) {
          drivetrain.setOdometry(new Pose2d( 1.76, kFieldY - 4.96, new Rotation2d(0)));
        }
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(0)), AutoPaths.get(0).getMarkers(), Constants.AUTO_EVENT_MAP),
      new SequentialCommandGroup(new InstantCommand(() -> arm.GrabGp(true)).andThen(new WaitCommand(0.1).andThen(new InstantCommand(() -> intake.PivotIn(false)))),
      new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(0.5), //Score High
      new InstantCommand(() -> arm.GrabGp(false))),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(1)), AutoPaths.get(1).getMarkers(), Constants.AUTO_EVENT_MAP),
      new ParallelCommandGroup(
        // new AlignWithTag(drivetrain, 2).withTimeout(0.7).andThen(
          new DriveForDistanceInDirection(drivetrain, 0.0, DriverStation.getAlliance() == Alliance.Red ? 28 / Constants.MeasurementConstants.kInchesToMeters: -28 / Constants.MeasurementConstants.kInchesToMeters).withTimeout(1.0),

        // new DriveToPoint(drivetrain, 1.82, DriverStation.getAlliance() == Alliance.Blue ? 3.82 : kFieldY - 3.82, 0.0, false), //FIXME
        new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(1.3)),
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, 4.0, 0.0, false, null));
      // new DriveForDistanceInDirection(drivetrain, 4.0, 0.0));

      //1.82, 3.82
  }   

  public static CommandBase ThreeGPCC(Drivetrain drivetrain, Arm arm, Intake intake, LEDs LEDs) {
    
    Constants.AUTO_EVENT_MAP.put("Collect", 
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        intake.CollectorOut(true);
        intake.PivotIn(false);
      }),
      new WaitCommand(0.9),
      new InstantCommand(() -> {
        intake.PivotIn(true);
      }),
      new WaitCommand(0.3),
      new InstantCommand(() ->intake.SetCollector(0, 0.3)),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake.SetCollector(0, 0.0))));
    
    Constants.AUTO_EVENT_MAP.put("IntakeUp", new InstantCommand(() -> {
      intake.CollectorOut(false);
      intake.SetCollector(0, 0.0);
    }));

    Constants.AUTO_EVENT_MAP.put("ZeroArm", new ArmToAngles(arm, 4.0, 0.0, false, null).withTimeout(3));

    Constants.AUTO_EVENT_MAP.put("Transfer", new SequentialCommandGroup(
      new ArmToAngles(arm, 4.0, -16.8, false, 0.3).withTimeout(0.8),
      new WaitCommand(0.1),
      new InstantCommand(() -> arm.GrabGp(true)),
      new WaitCommand(0.1),
      new InstantCommand(() -> intake.PivotIn(false)),
      new WaitCommand(0.1),
      new ArmToAngles(arm, -6.0, 90.0, true, 0.2).withTimeout(1.2)));


    Constants.AUTO_EVENT_MAP.put("ScoreLow", new SequentialCommandGroup(
      new InstantCommand(() -> intake.CollectorOut(true)),
      new WaitCommand(0.4),
      new InstantCommand(() -> intake.SetCollector(0, -0.94)),
      new WaitCommand(0.4),
      new InstantCommand(() -> {
        intake.SetCollector(0, 0.0);
        intake.CollectorOut(false);
      })
    ));
    
    // Constants.AUTO_EVENT_MAP.put("HoldArm", new ArmToAngles(arm, 3.0, 0.0, false, 0.3));

    List<PathPlannerTrajectory> AutoPaths = 
      PathPlanner.loadPathGroup("3GPCCHL", kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.setOdometry(new Pose2d( 1.84, 0.54, new Rotation2d(0.0)))),
      new ArmToAngles(arm, -8.0, 100.0, true, 0.15).withTimeout(0.8),
      // new ArmToAngles(arm, 20.0, 155.0, true, 0.15).withTimeout(0.8),
      new ArmToAngles(arm, 35.4, 150.0, true, 0.1).withTimeout(1.2), //Score High
      new InstantCommand(() -> arm.GrabGp(false)),
      new WaitCommand(0.1),
      new InstantCommand(() -> {
        if (DriverStation.getAlliance() == Alliance.Red) drivetrain.setOdometry(new Pose2d( 1.84, kFieldY - 0.54, new Rotation2d(0)));
      }),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(0)), AutoPaths.get(0).getMarkers(), Constants.AUTO_EVENT_MAP),
      // new ParallelCommandGroup(new AlignOnTag(drivetrain, 2).withTimeout(1),
        new SequentialCommandGroup(new InstantCommand(() -> arm.GrabGp(true)).andThen(new WaitCommand(0.1).andThen(new InstantCommand(() -> intake.PivotIn(false)))),
        new WaitCommand(0.1),
        new ArmToAngles(arm, 35.0, 155.0, true, 0.1).withTimeout(0.5), //Score High
        new InstantCommand(() -> arm.GrabGp(false))
        // )
        ),
      new FollowPathWithEvents(drivetrain.getCommandForTrajectory(AutoPaths.get(1)), AutoPaths.get(1).getMarkers(), Constants.AUTO_EVENT_MAP),
      new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(0.2),
      new InstantCommand(() -> {
        intake.CollectorOut(false);
        intake.SetCollector(0, 0.0);
      })),
      new DriveForDistanceInDirection(drivetrain, 2.0, 0.0)));

      
  }   
}
