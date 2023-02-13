// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import frc.robot.commands.AimAtNode;
import frc.robot.commands.AlignWithNode;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceRobotOnChargingStation;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Stinger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LEDs m_LEDs = new LEDs();
  // private final Arm m_arm = new Arm();
  private final Stinger m_Stinger = new Stinger();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);

  private final Joystick m_operatorStick =
      new Joystick(1);
  private Trigger button1 = new JoystickButton(m_operatorStick, 1);
  private Trigger button2 = new JoystickButton(m_operatorStick, 2);
  private Trigger button3 = new JoystickButton(m_operatorStick, 3);
  private Trigger button4 = new JoystickButton(m_operatorStick, 4);
  private Trigger button5 = new JoystickButton(m_operatorStick, 5);
  private Trigger button6 = new JoystickButton(m_operatorStick, 6);
  private Trigger button7 = new JoystickButton(m_operatorStick, 7);
  private Trigger button8 = new JoystickButton(m_operatorStick, 8);
  private Trigger button9 = new JoystickButton(m_operatorStick, 9);
  private Trigger button10 = new JoystickButton(m_operatorStick, 10);
  private Trigger button11 = new JoystickButton(m_operatorStick, 11);
  private Trigger button12 = new JoystickButton(m_operatorStick, 12);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticks(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis(), 
        m_driverController.y()
        )
    );

 
    // Configure the trigger bindings  
    configureBindings();

    // m_arm.setDefaultCommand(new FunctionalCommand(() -> {}, () -> 
    // {m_arm.setElbowMotors(m_operatorStick.getX());
    //   m_arm.setShoulderMotors(m_operatorStick.getY());},
    //    (interrupted) -> {m_arm.setShoulderMotors(0.0);
    //   m_arm.setElbowMotors(0.0);}, () -> false, m_arm));

    // m_arm.setDefaultCommand(new ArmToPos(m_arm, 0.0, 0.0));
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
    m_driverController.leftTrigger().whileTrue(new BalanceRobotOnChargingStation(m_drivetrain, () -> m_driverController.getLeftTriggerAxis()));


    // m_driverController.povUp().onTrue(new InstantCommand(() -> {m_Stinger.setSpoolMotor(0.1);}));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> {m_Stinger.setSpoolMotor(-0.1);}));
    // m_driverController.povUp().onFalse(new InstantCommand(() -> {m_Stinger.setSpoolMotor(0.0);}));
    // m_driverController.povDown().onFalse(new InstantCommand(() -> {m_Stinger.setSpoolMotor(0.0);}));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_drivetrain.updateOdometryIfTag()));

    // m_driverController.povLeft().whileTrue(new AlignWithNode(m_drivetrain, 1).andThen(new AimAtNode(m_drivetrain)));
    // m_driverController.povUp().whileTrue(new AlignWithNode(m_drivetrain, 2).andThen(new AimAtNode(m_drivetrain)));
    // m_driverController.povRight().whileTrue(new AlignWithNode(m_drivetrain, 3).andThen(new AimAtNode(m_drivetrain)));

    // m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_LEDs.setLEDS(Color.kYellow)));
    // m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_LEDs.setLEDS(Color.kPurple)));
    
    // m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_LEDs.LEDsOff()));
    // m_driverController.rightBumper().onFalse(new InstantCommand(() -> m_LEDs.LEDsOff()));

    // m_driverController.start().onTrue(new InstantCommand(() -> m_LEDs.toggleAmbulance()));

    // m_driverController.back().onTrue(new InstantCommand(() -> m_drivetrain.limelightToTapeMode()));
    // m_driverController.back().onFalse(new InstantCommand(() -> m_drivetrain.limelightToTagMode()));

    // button6.whileTrue(new ArmToPos(m_arm, 10.0, 90.0));
    // button5.whileTrue(new ArmToPos(m_arm, -10.0, -90.0));


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
