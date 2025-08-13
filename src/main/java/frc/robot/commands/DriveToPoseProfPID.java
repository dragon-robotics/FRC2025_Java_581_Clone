// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseProfPID extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final ApplyRobotSpeeds m_robotSpeeds;
  private final Pose2d m_targetPose;

  private final ProfiledPIDController m_translationController;
  private final ProfiledPIDController m_strafeController;
  private final ProfiledPIDController m_rotationController;

  /** Creates a new DriveToPoseProfPID. */
  public DriveToPoseProfPID(
      CommandSwerveDrivetrain swerve,
      ApplyRobotSpeeds robotSpeeds,
      Pose2d targetPose) {
    m_swerve = swerve;
    m_robotSpeeds = robotSpeeds;
    m_targetPose = targetPose;

    // Initialize the Profiled PID controllers
    m_translationController = new ProfiledPIDController(
        4.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(4.0, 8.0));
    // Initialize the Profiled PID controllers
    m_strafeController = new ProfiledPIDController(
        4.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(4.0, 8.0));
    // Initialize the Profiled PID controllers
    m_rotationController = new ProfiledPIDController(
        4.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(540),
            Units.degreesToRadians(720)));

    m_translationController.setTolerance(0.01);
    m_strafeController.setTolerance(0.01);
    m_rotationController.setTolerance(Units.degreesToRadians(2));

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Get the current pose and chassis speed of the robot
    Pose2d currentPose = m_swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = m_swerve.getState().Speeds;

    // Convert robot speeds to field speeds
    ChassisSpeeds currentFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        currentSpeeds, currentPose.getRotation());

    // Reset ProfiledPIDControllers with current position, velocity, and rotation
    m_translationController.reset(currentPose.getX());
    m_strafeController.reset(currentPose.getY());
    m_rotationController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(),
            currentFieldSpeeds.omegaRadiansPerSecond));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current pose of the robot
    Pose2d currentPose = m_swerve.getState().Pose;

    // Calculate translation, strafe, and rotation speeds
    double translationSpeed = m_translationController.calculate(
        m_swerve.getState().Pose.getX(), m_targetPose.getX());

    double strafeSpeed = m_strafeController.calculate(
        m_swerve.getState().Pose.getY(), m_targetPose.getY());

    double rotationSpeed = m_rotationController.calculate(
        m_swerve.getState().Pose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians());

    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        translationSpeed, strafeSpeed, rotationSpeed, currentPose.getRotation());

    // Set the robot speeds
    m_swerve.setControl(m_robotSpeeds.withSpeeds(targetSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    m_swerve.setControl(m_robotSpeeds.withSpeeds(new ChassisSpeeds())); // Send zero speeds
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_translationController.atGoal() &&
        m_strafeController.atGoal() &&
        m_rotationController.atGoal();
  }
}
