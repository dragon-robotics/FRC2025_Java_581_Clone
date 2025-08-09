package frc.robot;

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

public class Constants {

  public static final class FieldConstants {
    public static AprilTagFieldLayout APTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static class VisionConstants {
    // Set up 8 pose estimation cameras with their respective names and positions

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
}
