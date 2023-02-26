// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.Arm;
// import static frc.robot.Constants.ArmConstants.PID.*;
// import static frc.robot.Constants.ArmConstants.*;

// public class ArmToPos extends CommandBase {
//   private Arm m_arm;
//   private Double targetShoulderAngle, targetElbowAngle, targetXPos, targetYPos;
//   private CommandXboxController controller;

//   private PIDController shoulderController = new PIDController(kShoulderP, kShoulderI, kShoulderD);
//   private PIDController elbowController = new PIDController(kElbowP, kElbowI, kElbowD);
//   /** Creates a new ArmToPos. */
//   public ArmToPos(Arm arm, Double x, Double y, CommandXboxController Controller) {
//     m_arm = arm;
//     targetXPos = x;
//     targetYPos = y;
//     controller = Controller;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_arm);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }
  
//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   targetElbowAngle = m_arm.getIKElbow(targetXPos, targetYPos);
//   targetShoulderAngle = m_arm.getIKShoulder(targetXPos, targetYPos);
//   targetElbowAngle = targetElbowAngle > 155 || targetElbowAngle < -155 ? 0.0: targetElbowAngle;
//   targetShoulderAngle = targetShoulderAngle > 45 || targetShoulderAngle < -45 ? 0.0: targetShoulderAngle;
//   shoulderController.reset();
//   elbowController.reset();
//   elbowController.setSetpoint(targetElbowAngle);
//   shoulderController.setSetpoint(targetShoulderAngle);
//   elbowController.setTolerance(0.5);
//   shoulderController.setTolerance(0.5);
//   } 

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (Math.abs(controller.getLeftY()) > 0.2) {
//       targetXPos += Math.copySign(0.1, controller.getLeftY());
//     }
//     if (Math.abs(controller.getRightY()) > 0.2) {
//       targetYPos += Math.copySign(0.1, controller.getRightY());
//     }
//     targetElbowAngle = m_arm.getIKElbow(targetXPos, targetYPos);
//     targetShoulderAngle = m_arm.getIKShoulder(targetXPos, targetYPos);
//     targetElbowAngle = targetElbowAngle > 155 || targetElbowAngle < -155 ? 0.0: targetElbowAngle;
//     targetShoulderAngle = targetShoulderAngle > 45 || targetShoulderAngle < -45 ? 0.0: targetShoulderAngle;
//     elbowController.setSetpoint(targetElbowAngle);
//     shoulderController.setSetpoint(targetShoulderAngle);
//     var elbowSpeed = -elbowController.calculate(m_arm.getElbowAngle());
//     var shoulderSpeed = shoulderController.calculate(m_arm.getShoulderAngle());
//     elbowSpeed = MathUtil.clamp(elbowSpeed, -kElbowMaxSpeed, kElbowMaxSpeed);
//     shoulderSpeed = MathUtil.clamp(shoulderSpeed, -kShoulderMaxSpeed, kShoulderMaxSpeed);
//     m_arm.setElbowMotors(elbowSpeed);
//     m_arm.setShoulderMotors(-shoulderSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elbowController.close();
//     shoulderController.close();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
