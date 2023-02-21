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

public class ThrowCube extends CommandBase {
  private Arm m_arm;
  private Double clampSpeed, targetShoulderAngle, targetElbowAngle, releaseAngle;
  private CommandXboxController controller;
  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);

  /** Creates a new ArmToAngle. */
  public ThrowCube(Arm arm, Double shoulderAngle, Double elbowAngle, CommandXboxController incController, Double clamp, Double ReleaseAngle) {
    m_arm = arm;
    releaseAngle = ReleaseAngle;
    clampSpeed = clamp; // 0.15
    controller = incController;
    targetElbowAngle = elbowAngle > 165 || elbowAngle < -165 ? 0.0: elbowAngle;
    targetShoulderAngle = shoulderAngle > 40 || shoulderAngle < -40 ? 0.0: shoulderAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.GrabGp(true);
    shoulderController.reset();
    elbowController.reset();
    elbowController.setSetpoint(targetElbowAngle);
    shoulderController.setSetpoint(targetShoulderAngle);
    elbowController.setTolerance(0.5);
    shoulderController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_arm.getElbowAngle()) > releaseAngle) {
      m_arm.GrabGp(false);
    }
    if (Math.abs(controller.getLeftY()) > 0.2) {
      var prevAngle = targetElbowAngle;
      targetElbowAngle -= Math.copySign(0.2, controller.getLeftY());
      targetElbowAngle = targetElbowAngle > 165 || targetElbowAngle < -165 ? prevAngle: targetElbowAngle;
    }
    if (Math.abs(controller.getRightY()) > 0.2) {
      var prevAngle = targetShoulderAngle;
      targetShoulderAngle += Math.copySign(0.2, controller.getRightY());
      targetShoulderAngle = targetShoulderAngle > 37 || targetShoulderAngle < -37 ? prevAngle: targetShoulderAngle;
    }
    elbowController.setSetpoint(targetElbowAngle);
    shoulderController.setSetpoint(targetShoulderAngle);
    var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
    var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
    elbowSpeed = MathUtil.clamp(elbowSpeed, -2 * clampSpeed, 2 * clampSpeed);
    shoulderSpeed = MathUtil.clamp(shoulderSpeed, -clampSpeed, clampSpeed);
    m_arm.setElbowMotors(elbowSpeed);
    m_arm.setShoulderMotors(-shoulderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setElbowMotors(0.0);
    m_arm.setShoulderMotors(0.0);
    shoulderController.close();
    elbowController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (shoulderController.atSetpoint() && elbowController.atSetpoint()) {
    //   return true;
    // }
    return false;
  }
}
