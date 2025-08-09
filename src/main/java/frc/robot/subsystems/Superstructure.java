package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveMaintainHeadingCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Superstructure {
  /* Subsystems */
  private final CommandSwerveDrivetrain drivetrain;
  private final VisionSubsystem vision;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.RobotCentric driveRobotCentric;
  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      VisionSubsystem vision) {
    this.drivetrain = swerve;
    this.vision = vision;

    // Instantiate default field centric drive (no need to maintain heading) //
    drive = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
        .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate brake (X-lock swerve wheels) //
    brake = new SwerveRequest.SwerveDriveBrake();

    // Instantiate point (point swerve wheels in a specific direction) //
    point = new SwerveRequest.PointWheelsAt();

    // Instantiate robot centric drive (forward is based on forward pose of the
    // robot) //
    driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Instantiate field centric drive (maintain heading) //
    driveMaintainHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
        .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true);

    // Set the PID constants for the Maintain Heading controller //
    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP,
        SwerveConstants.HEADING_KI,
        SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Instantiate the Field and Robot Speeds Swerve Requests //
    applyFieldSpeeds
      = new SwerveRequest.ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds
      = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);    
  }

  public Command DriveMaintainHeading(
    DoubleSupplier translationSupplier,
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotationSupplier
  ) {

    return new DriveMaintainHeadingCommand(
        drivetrain,
        translationSupplier,
        strafeSupplier,
        rotationSupplier,
        drive,
        driveMaintainHeading);
  }
}