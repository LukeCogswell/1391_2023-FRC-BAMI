// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.Constants.ArmConstants.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;

public class HoldArmAtPos extends CommandBase {
  private Arm m_arm;
  private CommandXboxController m_controller;
  private Double targetShoulderAngle, targetElbowAngle, targetXPos, targetYPos, originalXPos, originalYPos;
  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);
  /** Creates a new HoldArmAtPos. */
  public HoldArmAtPos(Arm arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalXPos = m_arm.getFKArmPos()[0];
    originalYPos = m_arm.getFKArmPos()[1];
    targetXPos = originalXPos;
    targetYPos = originalYPos;

    targetElbowAngle = m_arm.getIKElbow(targetXPos, targetYPos);
    targetShoulderAngle = m_arm.getIKShoulder(targetXPos, targetYPos);
    targetElbowAngle = targetElbowAngle > 155 || targetElbowAngle < -155 ? 0.0: targetElbowAngle;
    targetShoulderAngle = targetShoulderAngle > 35.6 || targetShoulderAngle < -35.6 ? 0.0: targetShoulderAngle;
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
    if (Math.abs(m_controller.getLeftY()) > 0.2 || Math.abs(m_controller.getRightY()) > 0.2) {
      if (Math.abs(m_controller.getLeftY()) > 0.2) {
      var nextXPos = targetXPos - Math.copySign(0.2, m_controller.getLeftY());
      targetXPos = nextXPos > kMaxReach ? kMaxReach : nextXPos;
      targetXPos = nextXPos < -kMaxReach ? -kMaxReach : nextXPos;
      
      }
      if (Math.abs(m_controller.getRightY()) > 0.2) {
      var nextYPos = targetYPos - Math.copySign(0.2, m_controller.getRightY());
      targetYPos = nextYPos > kMaxHeight ? kMaxHeight : nextYPos;
      }
      targetElbowAngle = m_arm.getIKElbow(targetXPos, targetYPos);
      targetShoulderAngle = m_arm.getIKShoulder(targetXPos, targetYPos);
      
      targetElbowAngle = targetElbowAngle > 160 || targetElbowAngle < -160 ? m_arm.getElbowAngle(): targetElbowAngle;
      targetShoulderAngle = targetShoulderAngle > 35.5 || targetShoulderAngle < -35.5 ? m_arm.getShoulderAngle(): targetShoulderAngle;

      elbowController.setSetpoint(targetElbowAngle);
      shoulderController.setSetpoint(targetShoulderAngle);

  }
  
  var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
  var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
  
  elbowSpeed = MathUtil.clamp(elbowSpeed, -0.2, 0.2);
  shoulderSpeed = MathUtil.clamp(shoulderSpeed, -0.2, 0.2);
  
  // SAFETY IF
  if(elbowSpeed >= -0.2 && elbowSpeed <= 0.2 && shoulderSpeed >= -0.2 && shoulderSpeed <= 0.2) {
      m_arm.setElbowMotors(elbowSpeed);
      m_arm.setShoulderMotors(-shoulderSpeed);
  } else {
      m_arm.setElbowMotors(0.0);
      m_arm.setShoulderMotors(0.0);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    originalXPos = null;
    originalYPos = null;
    elbowController.close();
    shoulderController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
