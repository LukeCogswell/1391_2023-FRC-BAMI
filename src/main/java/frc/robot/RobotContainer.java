// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import frc.robot.commands.ArmToAngles;
import frc.robot.commands.Autos;
// import frc.robot.commands.BalanceRobotOnChargingStation;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.HoldArm;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
// import frc.robot.subsystems.Stinger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Compressor phCompressor = new Compressor(30, PneumaticsModuleType.REVPH);
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LEDs m_LEDs = new LEDs();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  // private final Stinger m_Stinger = new Stinger();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);

  private final Joystick m_operatorStick =
      new Joystick(1);
  Trigger button1 = new JoystickButton(m_operatorStick, 1);
  Trigger button2 = new JoystickButton(m_operatorStick, 2);
  Trigger button3 = new JoystickButton(m_operatorStick, 3);
  Trigger button4 = new JoystickButton(m_operatorStick, 4);
  Trigger button5 = new JoystickButton(m_operatorStick, 5);
  Trigger button6 = new JoystickButton(m_operatorStick, 6);
  Trigger button7 = new JoystickButton(m_operatorStick, 7);
  Trigger button8 = new JoystickButton(m_operatorStick, 8);
  Trigger button9 = new JoystickButton(m_operatorStick, 9);
  Trigger button10 = new JoystickButton(m_operatorStick, 10);
  Trigger button11 = new JoystickButton(m_operatorStick, 11);
  Trigger button12 = new JoystickButton(m_operatorStick, 12);

  Trigger povDownTrigger = new POVButton(m_operatorStick, 180);
  Trigger povRightTrigger = new POVButton(m_operatorStick, 90);
  Trigger povLeftTrigger = new POVButton(m_operatorStick, 270);
  Trigger povUpTrigger = new POVButton(m_operatorStick, 0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    phCompressor.enableDigital();
    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticks(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis(), 
        m_driverController.x()
        )
    );

 
    // Configure the trigger bindings  
    configureBindings();

    // m_arm.setDefaultCommand(new FunctionalCommand(() -> {}, () -> 
    // {m_arm.setElbowMotors(m_operatorStick.getX());
    //   m_arm.setShoulderMotors(m_operatorStick.getY());},
    //    (interrupted) -> {m_arm.setShoulderMotors(0.0);
    //   m_arm.setElbowMotors(0.0);}, () -> false, m_arm));

    m_arm.setDefaultCommand(new HoldArm(m_arm));
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
    
    /*************DRIVEBASE CONTROL***********/
    // m_driverController.povLeft().whileTrue(new AlignWithNode(m_drivetrain, 1).andThen(new AimAtNode(m_drivetrain)));
    // m_driverController.povUp().whileTrue(new AlignWithNode(m_drivetrain, 2).andThen(new AimAtNode(m_drivetrain)));
    // m_driverController.povRight().whileTrue(new AlignWithNode(m_drivetrain, 3).andThen(new AimAtNode(m_drivetrain)));

    // m_driverController.leftTrigger().whileTrue(new BalanceRobotOnChargingStation(m_drivetrain, () -> m_driverController.getLeftTriggerAxis()));
    
    // button1.onTrue(new InstantCommand(() -> {}, m_arm)); // removes operator control of arm
    /*************ARM CONTROL*****************/
    m_driverController.a().onTrue(new InstantCommand(() -> {m_arm.GrabGp(true);}));
    m_driverController.b().onTrue(new InstantCommand(() -> {m_arm.GrabGp(false);}));
    m_driverController.y().onTrue(new InstantCommand(() -> m_intake.CollectorOut(true)))
      .onFalse(new InstantCommand(() -> m_intake.CollectorOut(false)));
    
    // button1.onTrue(new InstantCommand(() -> {m_intake.PivotIn(false);}))
    // .onFalse(new InstantCommand(() -> m_intake.PivotIn(true)));
      // ^^^moves arm to position to grab from the collector on the floor
    
    
    
    // povUpTrigger.whileTrue(new ArmToAngles(m_arm, 36.0, 152.0));
    // povRightTrigger.whileTrue(new ArmToAngles(m_arm, 8.3, 94.0));
    // povLeftTrigger.whileTrue(new ArmToAngles(m_arm, -36.9, -31.0));
    // povDownTrigger.whileTrue(new ArmToAngles(m_arm, 0.0, 0.0));
    

    /*************INTAKE CONTROL*****************/
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_intake.SetCollector(0, 0.35)))
      .onFalse(new InstantCommand(() -> m_intake.SetCollector(0, 0.0)));
    
    m_driverController.povRight().onTrue(new InstantCommand(() -> m_intake.SetCollector(1, 0.5)).withTimeout(0.5)
    .andThen(new InstantCommand(() -> m_intake.SetCollector(0, 0.35))))
      .onFalse(new InstantCommand(() -> m_intake.SetCollector(0, 0.0)));
    
    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_intake.SetCollector(-1, 0.5)).withTimeout(0.5)
        .andThen(new InstantCommand(() -> m_intake.SetCollector(0, 0.35))))
      .onFalse(new InstantCommand(() -> m_intake.SetCollector(0, 0.0)));
    
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_intake.SetCollector(0, -1.0)))
      .onFalse(new InstantCommand(() -> m_intake.SetCollector(0, 0.0)));

    button6.onTrue(new InstantCommand(() -> m_intake.CollectorOut(true)));
    button4.onTrue(new InstantCommand(() -> m_intake.CollectorOut(false)));
    
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> {m_intake.PivotIn(false);}))
      .onFalse(new InstantCommand(() -> m_intake.PivotIn(true)));
    
    button9.whileTrue(new ArmToAngles(m_arm, -2.0, -12.5));

    button5.whileTrue(new InstantCommand(() -> m_arm.setElbowMotors(0.2)));
    button3.whileTrue(new InstantCommand(() -> m_arm.setElbowMotors(-0.2)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.ThreeGPBalanceNonCC(m_drivetrain, m_LEDs);
  }
}
