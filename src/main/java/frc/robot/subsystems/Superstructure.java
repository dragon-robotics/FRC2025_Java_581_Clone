package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveMaintainHeadingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Superstructure {
  /* Subsystems */
  private final CommandSwerveDrivetrain m_swerve;
  private final VisionSubsystem m_vision;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_fieldDrive;
  private final SwerveRequest.SwerveDriveBrake m_brakeDrive;
  private final SwerveRequest.PointWheelsAt m_pointDrive;
  private final SwerveRequest.RobotCentric m_robotDrive;
  private final SwerveRequest.FieldCentricFacingAngle m_fieldDriveFacingAngle;

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds;

  private final Telemetry logger;

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      VisionSubsystem vision) {
    m_swerve = swerve;
    m_vision = vision;

    // Instantiate default field centric drive (no need to maintain heading) //
    m_fieldDrive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
        .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate brake (X-lock swerve wheels) //
    m_brakeDrive = new SwerveRequest.SwerveDriveBrake();

    // Instantiate point (point swerve wheels in a specific direction) //
    m_pointDrive = new SwerveRequest.PointWheelsAt();

    // Instantiate robot centric drive (forward is based on forward pose of the
    // robot) //
    m_robotDrive = new SwerveRequest.RobotCentric()
        .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
        .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate field centric drive (maintain heading) //
    m_fieldDriveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
        .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true);

    // Set the PID constants for the Maintain Heading controller //
    m_fieldDriveFacingAngle.HeadingController.setPID(
        SwerveConstants.HEADING_KP,
        SwerveConstants.HEADING_KI,
        SwerveConstants.HEADING_KD);
    m_fieldDriveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    m_fieldDriveFacingAngle.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Instantiate the Field and Robot Speeds Swerve Requests //
    m_applyFieldSpeeds
      = new SwerveRequest.ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);
    m_applyRobotSpeeds
      = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);

    logger = new Telemetry(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_swerve.registerTelemetry(logger::telemeterize);
  }

  public Command DriveMaintainHeading(
    DoubleSupplier translationSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier
  ) {

    return new DriveMaintainHeadingCommand(
        m_swerve,
        translationSupplier,
        strafeSupplier,
        rotationSupplier,
        m_fieldDrive,
        m_fieldDriveFacingAngle);
  }
}