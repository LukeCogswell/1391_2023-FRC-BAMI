// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import java.util.Map;

import frc.robot.commands.AlignWithNode;
import frc.robot.commands.ArmToAngles;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveForDistanceInDirection;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.HoldArm;
import frc.robot.commands.ThrowCube;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import static frc.robot.Constants.MeasurementConstants.*;
// import frc.robot.subsystems.Stinger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Color kYellow = new Color(202, 198, 47);
  Compressor phCompressor = new Compressor(30, PneumaticsModuleType.REVPH);
  // private final ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LEDs m_LEDs = new LEDs();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  // private final Stinger m_Stinger = new Stinger();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    phCompressor.enableDigital();
    
    ShuffleboardLayout ArmControl = Shuffleboard.getTab("Commands")
    .getLayout("Gate", BuiltInLayouts.kList)
    .withSize(2,3)
    .withPosition(4,4)
    .withProperties(Map.of("Label Position", "LEFT"));
    GenericEntry armXEntry = 
      ArmControl.add("X Pos", 5.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 50.0))
        .getEntry();  
    GenericEntry armYEntry = 
      ArmControl.add("Y Pos", 8.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 5.0, "max", 50.0))
        .getEntry();  
    SuppliedValueWidget elbowAngle =
      ArmControl.addNumber("Elbow", 
      () -> m_arm.getIKElbow(
        armXEntry.getDouble(5.0),
        armYEntry.getDouble(8.0)));
        
    SuppliedValueWidget shoulderAngle =
      ArmControl.addNumber("Shoulder", 
      () -> m_arm.getIKShoulder(
        armXEntry.getDouble(5.0),
        armYEntry.getDouble(8.0)));
    


    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticks(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis(), 
        m_driverController.start()
        )
    );

 
    // Configure the trigger bindings  
    configureBindings();
    m_arm.setDefaultCommand(new HoldArm(m_arm, m_operatorController));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    /*************DRIVER CONTROLLER***********/
    
    m_driverController.x().onTrue(new InstantCommand(() -> m_LEDs.setLEDS(Color.kPurple))
    .andThen(new WaitCommand(5).andThen(new InstantCommand(() -> m_LEDs.setLEDS(Color.kRed)))));
    m_driverController.y().onTrue(new InstantCommand(() -> m_LEDs.setLEDS(Color.kGold))
    .andThen(new WaitCommand(5).andThen(new InstantCommand(() -> m_LEDs.setLEDS(Color.kRed)))));
    
    m_driverController.povLeft().whileTrue(new DriveForDistanceInDirection(m_drivetrain, 0.0, 22 / kInchesToMeters));
    m_driverController.povRight().whileTrue(new DriveForDistanceInDirection(m_drivetrain, 0.0, -22 / kInchesToMeters));
    // m_driverController.povLeft().whileTrue(new AlignWithNode(m_drivetrain, 1));
    m_driverController.povUp().whileTrue(new AlignWithNode(m_drivetrain, 2));
    // m_driverController.povRight().whileTrue(new AlignWithNode(m_drivetrain, 3));
    
    m_driverController.a().onTrue(new InstantCommand(() -> m_drivetrain.limelightToTagMode()));
    m_driverController.a().onTrue(new InstantCommand(() -> m_LEDs.intaking = !m_LEDs.intaking));
    
    /*************INTAKE CONTROL**********/
    
    m_driverController.povDown()
    .onTrue(new InstantCommand(() -> 
    m_intake.SetCollector(0, 0.3)))
    .onFalse(new InstantCommand(() -> 
    m_intake.SetCollector(0, 0.0)));
    
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> {
      m_intake.CollectorOut(true, m_LEDs);
      m_intake.PivotIn(false);
      m_intake.SetCollector(0, 0.35);
      })).onFalse(new InstantCommand(() -> {
        m_intake.PivotIn(true);
      }).andThen(new WaitCommand(0.5)).andThen(new InstantCommand(() -> 
        m_intake.SetCollector(0, 0.0))));
      
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> 
    m_intake.SetCollector(-1, 0.25)).andThen(new WaitCommand(0.3)).andThen(
      new InstantCommand(() -> m_intake.SetCollector(0, 0.35)).andThen(
      new WaitCommand(0.4).andThen(new InstantCommand(() -> m_intake.SetCollector(0, 0.0))))
      ));
      
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> 
    m_intake.SetCollector(1, 0.25)).andThen(new WaitCommand(0.3)).andThen(
      new InstantCommand(() -> m_intake.SetCollector(0, 0.35)).andThen(
      new WaitCommand(0.4).andThen(new InstantCommand(() -> m_intake.SetCollector(0, 0.0))))
      ));
        
    m_driverController.a().onTrue(new InstantCommand(() -> m_intake.CollectorOut(false, m_LEDs)));
        
    m_driverController.back()
    .onTrue(new InstantCommand(() -> 
    m_intake.SetCollector(2, 0.9)))
    .onFalse(new InstantCommand(() -> 
    m_intake.SetCollector(0, 0.0)));
        
    /******************LED CONTROL***************/
      
    m_driverController.y()
    .onTrue(new InstantCommand(
      () -> m_LEDs.setLEDS(Color.kPurple)
      )
    .andThen(new WaitCommand(5).andThen(
      () -> m_LEDs.setLEDS(Color.kBlack)
    )));

    m_driverController.x()
    .onTrue(new InstantCommand(
      () -> m_LEDs.setLEDS(Color.kPurple), m_LEDs
      )
    .andThen(new WaitCommand(5).andThen(
      () -> m_LEDs.setLEDS(Color.kBlack)
    )));


    /*************OPERATOR CONTROLLER*****************/
            
    m_operatorController.back().onTrue(new InstantCommand(() -> {}, m_arm));

    m_operatorController.y().whileTrue(
      new InstantCommand(() -> m_intake.PivotIn(false)).andThen(new WaitCommand(0.3)).andThen(
      new ArmToAngles(m_arm, -8.0, 90.0, true, 0.15).withTimeout(1.5)).andThen(
      new ArmToAngles(m_arm, 20.0, 150.0, true, 0.15).withTimeout(1.5).andThen(
      new ArmToAngles(m_arm, 36.0, 154.0, true, 0.12)))); //Score High

    m_operatorController.b().whileTrue(
      new InstantCommand(() -> m_intake.PivotIn(false)).andThen(new WaitCommand(0.3)).andThen(
      new ArmToAngles(m_arm, -8.0, 90.0, true, 0.15).withTimeout(1).andThen(
      new ArmToAngles(m_arm, 10.3, 98.0, true, 0.08)))); // Score Mid
    
    m_operatorController.a().whileTrue(
      new InstantCommand(() -> m_intake.PivotIn(false)).andThen(new WaitCommand(0.3)).andThen(
      new ArmToAngles(m_arm, 28.5, 37.4, true, 0.08))); // Score Low

    m_operatorController.x()
      .whileTrue(
        new InstantCommand(() -> m_intake.CollectorOut(false, m_LEDs)).andThen(
        new WaitCommand(0.5)).andThen(
        new ArmToAngles(m_arm, 4.2, -11.2, false, 0.15)));
          // new WaitCommand(0.2)).andThen(
          //   new InstantCommand(() -> m_intake.PivotIn(false))).andThen(
          //     new WaitCommand(0.5)).andThen(
          //       new ArmToAngles(m_arm, 3.0, 0.0, true, 0.15)
          //       ) );
                
    m_operatorController.povDown().whileTrue(
      new ArmToAngles(m_arm, 3.0, m_arm.getElbowAngle(), true, 0.2).withTimeout(1).andThen(
      new ArmToAngles(m_arm, 3.0, 0.0, true, 0.3))); // go to zero(straight up and down)
    
      m_operatorController.povUp().whileTrue(new ArmToAngles(m_arm, -10.0, -92.5, false, 0.15));
    
    m_operatorController.povRight().onTrue(new InstantCommand(() -> m_arm.GrabGp(false)));
    m_operatorController.povLeft().onTrue(new InstantCommand(() -> m_arm.GrabGp(true)));
    
    m_operatorController.rightTrigger()
    .onTrue(new ArmToAngles(
        m_arm, 
        m_arm.getShoulderAngle(), 
        m_arm.getElbowAngle() - 1.0,
        true, 0.2
      ).withTimeout(0.3))
    .onFalse(new ArmToAngles(m_arm, 0.0, m_arm.getElbowAngle(), false, 0.2));

    
    m_operatorController.start()
      .onTrue(new InstantCommand(() -> m_intake.PivotIn(false))) // backwards
      .onFalse(new InstantCommand(() -> m_intake.PivotIn(true)));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.OneGPBalance(m_drivetrain, m_arm, m_intake, m_LEDs);
  }
}
