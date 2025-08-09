package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {

  public static final class FieldConstants {
    public static AprilTagFieldLayout APTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static class VisionConstants {
    // Set up 8 pose estimation cameras with their respective names and positions
    public static final String[] APTAG_CAMERA_NAMES = {
        "AprilTagPoseEstCameraFL",
        "AprilTagPoseEstCameraF",
        "AprilTagPoseEstCameraFR",
        "AprilTagPoseEstCameraR",
        "AprilTagPoseEstCameraBR",
        "AprilTagPoseEstCameraB",
        "AprilTagPoseEstCameraBL",
        "AprilTagPoseEstCameraL"
    };

    // Front-Left Camera: Mounted at front-left corner, pointing outward at 45 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(45)));

    // Front Camera: Mounted at front face, pointing outward at 0 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_F_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(0),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(0)));
            
    // Front-Right Camera: Mounted at front-right corner, pointing outward at -45 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FR_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-45)));

    // Right Camera: Mounted at right face, pointing outward at -90 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_R_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-90)));

    // Back-Right Camera: Mounted at back-right corner, pointing outward at -135 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BR_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-135)));

    // Back Camera: Mounted at back face, pointing outward at -180 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_B_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(0),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-180)));

    // Back-Left Camera: Mounted at back-left corner, pointing outward at 135 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(135)));

    // Left Camera: Mounted at left face, pointing outward at 90 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_L_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(90)));

    public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
        APTAG_POSE_EST_CAM_FL_POS,
        APTAG_POSE_EST_CAM_F_POS,
        APTAG_POSE_EST_CAM_FR_POS,
        APTAG_POSE_EST_CAM_R_POS,
        APTAG_POSE_EST_CAM_BR_POS,
        APTAG_POSE_EST_CAM_B_POS,
        APTAG_POSE_EST_CAM_BL_POS,
        APTAG_POSE_EST_CAM_L_POS
    };

    // Vision standard deviation for pose estimation
    public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.3;
    public static double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDDEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STDDEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STDDEV_FACTORS = new double[] {
        1.0, // FL Camera
        1.0, // F Camera
        1.0, // FR Camera
        1.0, // R Camera
        1.0, // BR Camera
        1.0, // B Camera
        1.0, // BL Camera
        1.0 // L Camera
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available    
  }

  public static class SwerveConstants {
    // General constants for swerve drive //
    public static final double HEADING_KP = 3.5;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.5;
    public static final double HEADING_TOLERANCE = 0.01;

    public static final double MAX_SPEED_METERS_PER_SECOND = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double SWERVE_DEADBAND = 0.1;

    // SWERVE MODULE ODOMETRY STANDARD DEVIATIONS //
    public static final Matrix<N3, N1> ODOMETRY_STD = VecBuilder.fill(0.1, 0.1, 0.1);
  }

  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_BUTTON_PORT = 2;
    public static final int TEST_PORT = 3;
  }
  
  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }  
}
