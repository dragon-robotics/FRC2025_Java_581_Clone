package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Telemetry;
import frc.robot.commands.DriveMaintainHeadingCommand;
import frc.robot.commands.DriveToPoseProfPID;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  /* Used for Phoenix6 Logging */
  private final Telemetry logger;

  /** Creates a new Superstructure. */
  public Superstructure(CommandSwerveDrivetrain swerve, VisionSubsystem vision) {
    m_swerve = swerve;
    m_vision = vision;

    // Instantiate default field centric drive (no need to maintain heading) //
    m_fieldDrive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
            .withRotationalDeadband(
                SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05) // Add a 5% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate brake (X-lock swerve wheels) //
    m_brakeDrive = new SwerveRequest.SwerveDriveBrake();

    // Instantiate point (point swerve wheels in a specific direction) //
    m_pointDrive = new SwerveRequest.PointWheelsAt();

    // Instantiate robot centric drive (forward is based on forward pose of the
    // robot) //
    m_robotDrive =
        new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
            .withRotationalDeadband(
                SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05) // Add a 5% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate field centric drive (maintain heading) //
    m_fieldDriveFacingAngle =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(SwerveConstants.MAX_SPEED_METERS_PER_SECOND * 0.05)
            .withRotationalDeadband(SwerveConstants.MAX_ANGULAR_RATE_RADIANS_PER_SECOND * 0.05)
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true);

    // Set the PID constants for the Maintain Heading controller //
    m_fieldDriveFacingAngle.HeadingController.setPID(
        SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD);
    m_fieldDriveFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    m_fieldDriveFacingAngle.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Instantiate the Field and Robot Speeds Swerve Requests //
    m_applyFieldSpeeds =
        new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);
    m_applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);

    logger = new Telemetry(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_swerve.registerTelemetry(logger::telemeterize);
  }

  public Command DriveMaintainHeading(
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier halfSpeedSupplier) {

    return new DriveMaintainHeadingCommand(
        m_swerve,
        translationSupplier,
        strafeSupplier,
        rotationSupplier,
        halfSpeedSupplier,
        m_fieldDrive,
        m_fieldDriveFacingAngle);
  }

  private Pose2d calculateClosestPose(Pose2d currentPose, Pose2d[] targetPoses) {
    // Calculate the pose closest to the current pose
    Pose2d closestPose = null;
    double minDistanceSq = Double.MAX_VALUE; // Use squared distance to avoid sqrt

    // Iterate through the list of target poses
    for (Pose2d targetPose : targetPoses) {
      Transform2d translationDelta = targetPose.minus(currentPose);

      // Calculate the squared distance between the translations
      double distanceSq = translationDelta.getTranslation().getNorm();

      // If this pose is closer than the current minimum, update
      if (distanceSq < minDistanceSq) {
        minDistanceSq = distanceSq;
        closestPose = targetPose;
      }
    }

    return closestPose;
  }

  public Command DriveToClosestReefBranchPoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d[] targetPoses =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.Reef.RED_REEF_BRANCHES
                  : FieldConstants.Reef.BLUE_REEF_BRANCHES;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, targetPoses);

          // Add intermediate waypoint (1 meter back from target)
          Transform2d backwardOffset = new Transform2d(-0.25, 0.0, Rotation2d.kZero);

          return new DriveToPoseProfPID(
                  m_swerve,
                  m_applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(
                      Units.degreesToRadians(540), Units.degreesToRadians(720)))
              .andThen(
                  new DriveToPoseProfPID(
                      m_swerve,
                      m_applyRobotSpeeds,
                      closestPose,
                      new TrapezoidProfile.Constraints(4.0, 8.0),
                      new TrapezoidProfile.Constraints(4.0, 8.0),
                      new TrapezoidProfile.Constraints(
                          Units.degreesToRadians(540), Units.degreesToRadians(720))));
        },
        Set.of(m_swerve));
  }

  public Command DriveToClosestReefAlgaePoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d[] targetPoses =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.Reef.RED_REEF_ALGAE
                  : FieldConstants.Reef.BLUE_REEF_ALGAE;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, targetPoses);

          // Add intermediate waypoint (0.25 meter back from target)
          Transform2d backwardOffset = new Transform2d(-0.25, 0.0, Rotation2d.kZero);

          return new DriveToPoseProfPID(
                  m_swerve,
                  m_applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(
                      Units.degreesToRadians(540), Units.degreesToRadians(720)))
              .andThen(
                  new DriveToPoseProfPID(
                      m_swerve,
                      m_applyRobotSpeeds,
                      closestPose,
                      new TrapezoidProfile.Constraints(4.0, 8.0),
                      new TrapezoidProfile.Constraints(4.0, 8.0),
                      new TrapezoidProfile.Constraints(
                          Units.degreesToRadians(540), Units.degreesToRadians(720))));
        },
        Set.of(m_swerve));
  }

  public Command DriveToClosestCoralStationPoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d[] targetPoses =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.CoralStation.RED_CORAL_STATION_LOCS
                  : FieldConstants.CoralStation.BLUE_CORAL_STATION_LOCS;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, targetPoses);

          return new DriveToPoseProfPID(
              m_swerve,
              m_applyRobotSpeeds,
              closestPose,
              new TrapezoidProfile.Constraints(4.0, 8.0),
              new TrapezoidProfile.Constraints(4.0, 8.0),
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(540), Units.degreesToRadians(720)));
        },
        Set.of(m_swerve));
  }

  public Command DriveToProcessorPoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d targetPose =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.Processor.RED_PROCESSOR_LOC
                  : FieldConstants.Processor.BLUE_PROCESSOR_LOC;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, new Pose2d[] {targetPose});

          // Add intermediate waypoint (0.25 meter back from target)
          Transform2d backwardOffset = new Transform2d(-0.5, 0.0, Rotation2d.kZero);

          return new DriveToPoseProfPID(
                  m_swerve,
                  m_applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(
                      Units.degreesToRadians(540), Units.degreesToRadians(720)))
              .andThen(
                  new DriveToPoseProfPID(
                      m_swerve,
                      m_applyRobotSpeeds,
                      closestPose,
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(
                          Units.degreesToRadians(540), Units.degreesToRadians(720))));
        },
        Set.of(m_swerve));
  }

  public Command DriveToClosestBargePoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d[] targetPoses =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.Barge.RED_BARGE_LOCS
                  : FieldConstants.Barge.BLUE_BARGE_LOCS;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, targetPoses);

          // Add intermediate waypoint (0.25 meter back from target)
          Transform2d backwardOffset = new Transform2d(-1.0, 0.0, Rotation2d.kZero);

          return new DriveToPoseProfPID(
                  m_swerve,
                  m_applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(4.0, 8.0),
                  new TrapezoidProfile.Constraints(
                      Units.degreesToRadians(540), Units.degreesToRadians(720)))
              .andThen(
                  new DriveToPoseProfPID(
                      m_swerve,
                      m_applyRobotSpeeds,
                      closestPose,
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(
                          Units.degreesToRadians(540), Units.degreesToRadians(720))));
        },
        Set.of(m_swerve));
  }

  public Command DriveToClosestCagePoseCommand() {
    return new DeferredCommand(
        () -> {
          // Grab the robot's current alliance
          Optional<Alliance> alliance = DriverStation.getAlliance();

          // Grab the robot's current pose
          Pose2d currentPose = m_swerve.getState().Pose;

          // Initialize the target poses based on the alliance and whether we are left or
          // right //
          Pose2d[] targetPoses =
              alliance.isPresent() && (alliance.get() == Alliance.Red)
                  ? FieldConstants.Cage.RED_CAGE_LOCS
                  : FieldConstants.Cage.BLUE_CAGE_LOCS;

          // Calculate the pose closest to the current pose
          Pose2d closestPose = calculateClosestPose(currentPose, targetPoses);

          // Add intermediate waypoint (0.25 meter back from target)
          Transform2d backwardOffset = new Transform2d(-1.5, 0.0, Rotation2d.kZero);

          return new DriveToPoseProfPID(
                  m_swerve,
                  m_applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset),
                  new TrapezoidProfile.Constraints(2.0, 8.0),
                  new TrapezoidProfile.Constraints(2.0, 8.0),
                  new TrapezoidProfile.Constraints(
                      Units.degreesToRadians(540), Units.degreesToRadians(720)))
              .andThen(
                  new DriveToPoseProfPID(
                      m_swerve,
                      m_applyRobotSpeeds,
                      closestPose,
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(1.0, 8.0),
                      new TrapezoidProfile.Constraints(
                          Units.degreesToRadians(540), Units.degreesToRadians(720))));
        },
        Set.of(m_swerve));
  }
}
