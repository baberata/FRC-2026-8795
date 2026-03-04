package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Simple Vision class - detects AprilTags and outputs all data to Shuffleboard.
 */
public class Vision extends SubsystemBase
{
  // Camera
  private final PhotonCamera camera;
  private PhotonPipelineResult latestResult = null;
  
  // Camera position on robot (for reference)
  // X = forward/back from center, Y = left(+)/right(-), Z = height
  private static final double CAMERA_OFFSET_X = 0.0;      // meters forward from center
  private static final double CAMERA_OFFSET_Y = -0.27;    // meters (0.27m to the RIGHT = negative)
  private static final double CAMERA_HEIGHT = 0.19;       // meters (19cm above ground)

  public Vision()
  {
    camera = new PhotonCamera("center");
  }

  @Override
  public void periodic()
  {
    // Get all unread results and use the most recent one
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (!results.isEmpty())
    {
      latestResult = results.get(results.size() - 1); // Get most recent
    }
    
    // Check if we have a valid result and it has targets
    boolean hasTargets = latestResult != null && latestResult.hasTargets();
    SmartDashboard.putBoolean("Vision/HasTarget", hasTargets);
    
    if (hasTargets)
    {
      // Get the best target
      PhotonTrackedTarget target = latestResult.getBestTarget();
      
      // === RAW CAMERA DATA ===
      SmartDashboard.putNumber("Vision/Yaw (deg)", target.getYaw());
      SmartDashboard.putNumber("Vision/Pitch (deg)", target.getPitch());
      SmartDashboard.putNumber("Vision/Area (%)", target.getArea());
      SmartDashboard.putNumber("Vision/Skew (deg)", target.getSkew());
      SmartDashboard.putNumber("Vision/TagID", target.getFiducialId());
      SmartDashboard.putNumber("Vision/Ambiguity", target.getPoseAmbiguity());
      
      // === 3D TRANSFORM DATA (Camera to Target) ===
      Transform3d cameraToTarget = target.getBestCameraToTarget();
      
      // Camera-relative distances
      double camDistX = cameraToTarget.getX();  // forward from camera
      double camDistY = cameraToTarget.getY();  // left(+)/right(-) from camera
      double camDistZ = cameraToTarget.getZ();  // up(+)/down(-) from camera
      
      SmartDashboard.putNumber("Vision/Cam Distance X (m)", camDistX);
      SmartDashboard.putNumber("Vision/Cam Distance Y (m)", camDistY);
      SmartDashboard.putNumber("Vision/Cam Distance Z (m)", camDistZ);
      
      // === ROBOT-RELATIVE DISTANCES (incorporating camera offset) ===
      // Target position relative to robot center
      double robotDistX = camDistX + CAMERA_OFFSET_X;  // forward from robot center
      double robotDistY = camDistY + CAMERA_OFFSET_Y;  // left/right from robot center
      double robotDistZ = camDistZ + CAMERA_HEIGHT;    // height from ground
      
      SmartDashboard.putNumber("Vision/Robot Distance X (m)", robotDistX);
      SmartDashboard.putNumber("Vision/Robot Distance Y (m)", robotDistY);
      SmartDashboard.putNumber("Vision/Robot Distance Z (m)", robotDistZ);
      
      // Total distance from robot center to target (horizontal plane only)
      double robotHorizontalDist = Math.sqrt(robotDistX * robotDistX + robotDistY * robotDistY);
      SmartDashboard.putNumber("Vision/Robot Horizontal Dist (m)", robotHorizontalDist);
      
      // Total 3D distance from robot center to target
      double robotTotalDist = Math.sqrt(robotDistX * robotDistX + robotDistY * robotDistY + robotDistZ * robotDistZ);
      SmartDashboard.putNumber("Vision/Robot Total Dist (m)", robotTotalDist);
      
      // Angle to target from robot center (in horizontal plane)
      double robotAngleToTarget = Math.toDegrees(Math.atan2(robotDistY, robotDistX));
      SmartDashboard.putNumber("Vision/Robot Angle to Target (deg)", robotAngleToTarget);
      
      // Target rotation
      SmartDashboard.putNumber("Vision/Target Yaw Rot (deg)", Math.toDegrees(cameraToTarget.getRotation().getZ()));
      SmartDashboard.putNumber("Vision/Target Pitch Rot (deg)", Math.toDegrees(cameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Vision/Target Roll Rot (deg)", Math.toDegrees(cameraToTarget.getRotation().getX()));
      
      // Timing
      SmartDashboard.putNumber("Vision/Latency (ms)", latestResult.metadata.getLatencyMillis());
      SmartDashboard.putNumber("Vision/Target Count", latestResult.getTargets().size());
    }
    else
    {
      SmartDashboard.putNumber("Vision/Yaw (deg)", 0);
      SmartDashboard.putNumber("Vision/Pitch (deg)", 0);
      SmartDashboard.putNumber("Vision/TagID", -1);
      SmartDashboard.putNumber("Vision/Robot Horizontal Dist (m)", 0);
      SmartDashboard.putNumber("Vision/Robot Angle to Target (deg)", 0);
      SmartDashboard.putNumber("Vision/Target Count", 0);
    }
  }

  public PhotonCamera getCamera() { return camera; }
  
  public boolean hasTarget()
  {
    return latestResult != null && latestResult.hasTargets();
  }
  
  public PhotonTrackedTarget getBestTarget()
  {
    if (latestResult != null && latestResult.hasTargets())
    {
      return latestResult.getBestTarget();
    }
    return null;
  }
}