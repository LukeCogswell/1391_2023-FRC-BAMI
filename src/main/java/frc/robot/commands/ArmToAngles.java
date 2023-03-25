// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import static frc.robot.Constants.ArmConstants.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;

public class ArmToAngles extends CommandBase {
  private Arm m_arm;
  private Boolean grabGP;
  private Double targetShoulderAngle, targetElbowAngle, shoulderSpeedMultiplier, elbowSpeedMultiplier;
  private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
  private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);
  /** Creates a new ArmToAngle. */

  /**
   * Moves Arm to designated angles. 
   * @param arm - the current {@link Arm}
   * @param shoulderAngle - desired angle of shoulder joint
   * @param elbowAngle - desired angle of elbow joint
   * @param GRABGP - whether to open or close the gripper during the command
   * @param clamp - clamp speed of shoulder. elbow is clamped to 2x this.
   * 
   * 
   */
  public ArmToAngles(Arm arm, Double shoulderAngle, Double elbowAngle, Boolean GRABGP, Double clamp) {
    grabGP = GRABGP;
    m_arm = arm;
    targetElbowAngle = elbowAngle > 165 || elbowAngle < -165 ? 0.0: elbowAngle;
    targetShoulderAngle = shoulderAngle > 40 || shoulderAngle < -40 ? 0.0: shoulderAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // speedMultiplier = Math.abs(m_arm.getFK(targetShoulderAngle, targetElbowAngle)[0]) >  20 ? 0.5 : 1; 
    m_arm.GrabGp(grabGP);
    shoulderController.reset();
    elbowController.reset();
    elbowController.setSetpoint(targetElbowAngle);
    shoulderController.setSetpoint(targetShoulderAngle);
    elbowController.setTolerance(0.5);
    shoulderController.setTolerance(0.5);
    shoulderSpeedMultiplier = 1.0;
    elbowSpeedMultiplier = 1.0;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speedMultiplier = Math.abs(m_arm.getFKArmPos()[0]) >  30 ? 0.7 : 1; 
    if (Math.abs(m_arm.getShoulderAngle()) > 20 && Math.abs(targetShoulderAngle) > 20) {
      shoulderSpeedMultiplier = 0.5;
    }
    if (Math.abs(m_arm.getShoulderAngle()) > 25 && Math.abs(targetShoulderAngle) > 25) {
      shoulderSpeedMultiplier = 0.3;
    }
    if (Math.abs(m_arm.getShoulderAngle()) > 30 && Math.abs(targetShoulderAngle) > 30) {
      shoulderSpeedMultiplier = 0.1;
    }
    if (Math.abs(m_arm.getElbowAngle()) > 145 && Math.abs(targetElbowAngle) > 145) {
      elbowSpeedMultiplier = 0.9;
    }
    if (Math.abs(m_arm.getElbowAngle()) > 150 && Math.abs(targetElbowAngle) > 150) {
      elbowSpeedMultiplier = 0.8;
    }
    if (Math.abs(m_arm.getElbowAngle()) > 155 && Math.abs(targetElbowAngle) > 155) {
      elbowSpeedMultiplier = 0.6;
    }
    
    var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
    var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
    elbowSpeed = MathUtil.clamp(elbowSpeed, -kElbowMaxSpeed, kElbowMaxSpeed);
    shoulderSpeed = MathUtil.clamp(shoulderSpeed, -kShoulderMaxSpeed, kShoulderMaxSpeed);
    m_arm.setElbowMotors(elbowSpeed * elbowSpeedMultiplier);
    m_arm.setShoulderMotors(-shoulderSpeed * shoulderSpeedMultiplier);
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
    return false;
  }
}
