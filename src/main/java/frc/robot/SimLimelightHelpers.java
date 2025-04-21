package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * A simulation-specific implementation of LimelightHelpers that provides
 * simulated vision data during simulation.
 */
public class SimLimelightHelpers {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable table = inst.getTable("limelight");
    
    // Publishers for simulated Limelight data
    private static final DoubleArrayPublisher botposePub = table.getDoubleArrayTopic("botpose").publish();
    private static final DoubleArrayPublisher botpose_wpiredPub = table.getDoubleArrayTopic("botpose_wpired").publish();
    private static final DoubleArrayPublisher botpose_wpibluePub = table.getDoubleArrayTopic("botpose_wpiblue").publish();
    private static final DoubleArrayPublisher camerapose_robotspacePub = table.getDoubleArrayTopic("camerapose_robotspace").publish();
    private static final StringPublisher jsonPub = table.getStringTopic("json").publish();
    
    // Simulated target visibility
    private static boolean targetVisible = true;
    
    // Simulated target ID
    private static int targetId = 1;
    
    /**
     * Sets whether a target is visible in the simulated Limelight
     * @param visible Whether a target should be visible
     */
    public static void setTargetVisible(boolean visible) {
        targetVisible = visible;
    }
    
    /**
     * Sets the simulated target ID
     * @param id The target ID to simulate
     */
    public static void setTargetId(int id) {
        targetId = id;
    }
    
    /**
     * Simulates the getTV() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return Whether a target is visible
     */
    public static boolean getTV(String limelightName) {
        return targetVisible;
    }
    
    /**
     * Simulates the getBotPose() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose(String limelightName) {
        // Default pose if no target is visible
        if (!targetVisible) {
            return new double[]{0, 0, 0, 0, 0, 0};
        }
        
        // Simulate a pose based on the target ID
        // This is just an example - you can modify this to provide different poses
        double x = 0;
        double y = 0;
        double z = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        
        switch (targetId) {
            case 1: // Left reef tag
                x = 2.0;
                y = 2.0;
                yaw = 45;
                break;
            case 2: // Right reef tag
                x = 2.0;
                y = -2.0;
                yaw = -45;
                break;
            case 3: // Higher tag
                x = 3.0;
                y = 0.0;
                yaw = 0;
                break;
            default:
                // Default pose
                x = 1.0;
                y = 1.0;
                yaw = 30;
                break;
        }
        
        double[] pose = {x, y, z, roll, pitch, yaw};
        botposePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose_wpiRed() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space for the red alliance [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose_wpiRed(String limelightName) {
        double[] pose = getBotPose(limelightName);
        botpose_wpiredPub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space for the blue alliance [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose_wpiBlue(String limelightName) {
        double[] pose = getBotPose(limelightName);
        botpose_wpibluePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getCameraPose_RobotSpace() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The camera's pose in robot space [x, y, z, roll, pitch, yaw]
     */
    public static double[] getCameraPose_RobotSpace(String limelightName) {
        // Simulate camera position relative to robot
        double[] pose = {0.5, 0, 0.5, 0, 0, 0};
        camerapose_robotspacePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose2d() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space
     */
    public static Pose2d getBotPose2d(String limelightName) {
        double[] poseArray = getBotPose(limelightName);
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    /**
     * Simulates the getBotPose2d_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space for the blue alliance
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        double[] poseArray = getBotPose_wpiBlue(limelightName);
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    /**
     * Simulates the getBotPose2d_wpiRed() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space for the red alliance
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        double[] poseArray = getBotPose_wpiRed(limelightName);
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    /**
     * Simulates the getBotPose3d() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 3D pose in field space
     */
    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getBotPose(limelightName);
        return new Pose3d(
            new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
            new Rotation3d(
                Units.degreesToRadians(poseArray[3]),
                Units.degreesToRadians(poseArray[4]),
                Units.degreesToRadians(poseArray[5])
            )
        );
    }
    
    /**
     * Simulates the getBotPoseEstimate_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return A simulated pose estimate for the blue alliance
     */
    public static LimelightHelpers.PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        if (!targetVisible) {
            return new LimelightHelpers.PoseEstimate();
        }
        
        double[] poseArray = getBotPose_wpiBlue(limelightName);
        Pose2d pose = new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
        
        // Create a simulated pose estimate
        return new LimelightHelpers.PoseEstimate(
            pose,
            System.currentTimeMillis() / 1000.0, // Current time in seconds
            0.02, // 20ms latency
            1, // Tag count
            0.5, // Tag span
            1.0, // Average tag distance
            0.1, // Average tag area
            new LimelightHelpers.RawFiducial[0], // No raw fiducials
            false // Not MegaTag2
        );
    }
    
    /**
     * Simulates the getBotPoseEstimate_wpiBlue_MegaTag2() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return A simulated pose estimate for the blue alliance using MegaTag2
     */
    public static LimelightHelpers.PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        if (!targetVisible) {
            return new LimelightHelpers.PoseEstimate();
        }
        
        double[] poseArray = getBotPose_wpiBlue(limelightName);
        Pose2d pose = new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
        
        // Create a simulated pose estimate
        return new LimelightHelpers.PoseEstimate(
            pose,
            System.currentTimeMillis() / 1000.0, // Current time in seconds
            0.02, // 20ms latency
            1, // Tag count
            0.5, // Tag span
            1.0, // Average tag distance
            0.1, // Average tag area
            new LimelightHelpers.RawFiducial[0], // No raw fiducials
            true // MegaTag2
        );
    }
    
    /**
     * Simulates the SetRobotOrientation() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @param yaw Robot yaw in degrees
     * @param yawRate Angular velocity of robot yaw in degrees per second
     * @param pitch Robot pitch in degrees
     * @param pitchRate Angular velocity of robot pitch in degrees per second
     * @param roll Robot roll in degrees
     * @param rollRate Angular velocity of robot roll in degrees per second
     */
    public static void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
        double pitch, double pitchRate, double roll, double rollRate) {
        // In simulation, we don't need to do anything with this data
        // But we'll publish it to NetworkTables for consistency
        double[] orientation = {yaw, yawRate, pitch, pitchRate, roll, rollRate};
        table.getDoubleArrayTopic("robot_orientation_set").publish().set(orientation);
    }
} 