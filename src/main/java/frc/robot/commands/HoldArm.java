// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.PID.*;
import static frc.robot.Constants.ArmConstants.*;

public class HoldArm extends CommandBase {
  Arm m_arm;
  private Double targetElbowAngle, targetShoulderAngle;
  private CommandXboxController controller;
  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);
  /** Creates a new HoldArm. */
  /*
   *  Holds arm at cuurrent position
   * 
   *  @param Controller - which controller to reference for stick values for incremental adjustments 
   * 
   * 
   */
  public HoldArm(Arm arm, CommandXboxController Controller) {
    m_arm = arm;
    controller = Controller;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderController.reset();
    elbowController.reset();
    targetShoulderAngle = m_arm.getShoulderAngle();
    targetElbowAngle = m_arm.getElbowAngle(); 
    shoulderController.setSetpoint(targetShoulderAngle);
    elbowController.setSetpoint(targetElbowAngle);
    shoulderController.enableContinuousInput(-180.0, 180.0);
    elbowController.enableContinuousInput(-180.0, 180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(controller.getLeftY()) > 0.2) {
      var prevAngle = targetElbowAngle;
      targetElbowAngle -= Math.copySign(0.3, controller.getLeftY());
      targetElbowAngle = targetElbowAngle > 165 || targetElbowAngle < -165 ? prevAngle: targetElbowAngle;
      elbowController.setSetpoint(targetElbowAngle);
    }
    if (Math.abs(controller.getRightY()) > 0.2) {
      var prevAngle = targetShoulderAngle;
      targetShoulderAngle += Math.copySign(0.3, controller.getRightY());
      targetShoulderAngle = targetShoulderAngle > 37 || targetShoulderAngle < -37 ? prevAngle: targetShoulderAngle;
      shoulderController.setSetpoint(targetShoulderAngle);
    }
    
    var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
    var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
    elbowSpeed = MathUtil.clamp(elbowSpeed, -kElbowMaxSpeed, kElbowMaxSpeed);
    shoulderSpeed = MathUtil.clamp(shoulderSpeed, -kShoulderMaxSpeed, kShoulderMaxSpeed);

    m_arm.setElbowMotors(elbowSpeed);
    m_arm.setShoulderMotors(-shoulderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setElbowMotors(0.0);
    m_arm.setShoulderMotors(0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
