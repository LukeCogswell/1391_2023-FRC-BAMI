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
import static frc.robot.Constants.MeasurementConstants.*;

public class ArmToPos extends CommandBase {
  private Arm m_arm;
  private Double targetShoulderAngle, targetElbowAngle, targetXPos, targetYPos, originalXPos, originalYPos;
  private CommandXboxController controller;

  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);
  /** Creates a new ArmToPos. */
  public ArmToPos(Arm arm, Double x, Double y, CommandXboxController Controller) {
    m_arm = arm;
    originalXPos = x;
    originalYPos = y;
    targetXPos = x;
    targetYPos = y;
    controller = Controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    if (Math.abs(controller.getLeftY()) > 0.2 || Math.abs(controller.getRightY()) > 0.2) {
        if (Math.abs(controller.getLeftY()) > 0.2) {
        var nextXPos = targetXPos - Math.copySign(0.2, controller.getLeftY());
        targetXPos = nextXPos > kMaxReach ? kMaxReach : nextXPos;
        targetXPos = nextXPos < -kMaxReach ? -kMaxReach : nextXPos;
        
        }
        if (Math.abs(controller.getRightY()) > 0.2) {
        var nextYPos = targetYPos - Math.copySign(0.2, controller.getRightY());
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
    if (interrupted) {
        targetXPos = originalXPos;
        targetYPos = originalYPos;
    } 
    elbowController.close();
    shoulderController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
