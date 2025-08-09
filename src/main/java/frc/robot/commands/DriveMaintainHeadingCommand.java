// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveMaintainHeadingCommand extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;

  private SwerveRequest.FieldCentric m_fieldDrive;
  private SwerveRequest.FieldCentricFacingAngle m_fieldDriveFacingAngle;

  /** The last time the rotation was triggered, used to determine if the robot is actively rotating. */
  private double rotationLastTriggered;

  private Optional<Rotation2d> currentHeading;

  /** Creates a new DriveMaintainingHeadingCommand. */
  public DriveMaintainHeadingCommand(
    CommandSwerveDrivetrain drivetrain,
    DoubleSupplier translationSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier,
    SwerveRequest.FieldCentric fieldDrive,
    SwerveRequest.FieldCentricFacingAngle fieldDriveFacingAngle
  ) {
    m_drivetrain = drivetrain;
    m_translationSupplier = translationSupplier;
    m_strafeSupplier = strafeSupplier;
    m_rotationSupplier = rotationSupplier;

    m_fieldDrive = fieldDrive;
    m_fieldDriveFacingAngle = fieldDriveFacingAngle;

    rotationLastTriggered = Timer.getFPGATimestamp();

    currentHeading = Optional.empty();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rawTranslation = m_translationSupplier.getAsDouble();
    double rawStrafe = m_strafeSupplier.getAsDouble();
    double rawRotation = m_rotationSupplier.getAsDouble();

    // Apply deadband to all inputs
    double translation = MathUtil.applyDeadband(rawTranslation, 0.1, 1);
    double strafe = MathUtil.applyDeadband(rawStrafe, 0.1, 1);
    double rotation = MathUtil.applyDeadband(rawRotation, 0.1, 1);

    // Apply a cubic curve to the inputs for smoother control
    translation = translation * translation * translation;
    strafe = strafe * strafe * strafe;
    rotation = rotation * rotation * rotation;

    // Scale the inputs to the maximum speed and angular rate
    translation *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    strafe *= SwerveConstants.MAX_SPEED_METERS_PER_SECOND;
    rotation *= SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND;

    // Check for active rotation or active rotation input //
    boolean rotationTriggered = Math.abs(rawRotation) > SwerveConstants.SWERVE_DEADBAND;
    if (rotationTriggered) {
      rotationLastTriggered = Timer.getFPGATimestamp();
    }

    // Active rotation = rotation triggered in the last 100ms and greater than 10deg/s angular speed
    boolean rotationActive =
      MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1) &&
      (Math.abs(m_drivetrain.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(10));

    if(rotationTriggered || rotationActive){
      // If the rotation is triggered or active, we need to set the current heading null
      currentHeading = Optional.empty();
      m_drivetrain.setControl(
        m_fieldDrive
          .withVelocityX(translation)
          .withVelocityY(strafe)
          .withRotationalRate(rotation)
      );
    } else {
      // If the rotation is not triggered, we can use the current heading
      if (currentHeading.isEmpty()) {
        // If the current heading is not set, we can use the drivetrain's current heading
        currentHeading = Optional.of(m_drivetrain.getState().Pose.getRotation());
      }

      // Grab the alliance color and adjust the current heading accordingly
      currentHeading =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ?
          // If the alliance color is red, we need to flip the heading
          Optional.of(m_drivetrain.getState().Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))) :
          Optional.of(m_drivetrain.getState().Pose.getRotation());

      m_drivetrain.setControl(
        m_fieldDriveFacingAngle
          .withVelocityX(translation)
          .withVelocityY(strafe)
          .withTargetDirection(currentHeading.get())
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
