package frc.robot.Commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Vision.Vision;

public class AlignToGoalCommand extends Command{
    private Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;
    public AlignToGoalCommand (Vision vision) {
        this.poseSupplier = () -> vision.getLocation(); //replace with function for getting position
        addRequirements(vision);
    }
    public AlignToGoalCommand (Robotstate robotState) {
        this.poseSupplier = () -> robotState.getRobotPoseFromSwerveDriveOdometry();
    }

    private Pose2d getTargetPose() {
        targetPose = poseSupplier.get();

    }

    public static Pose2d getTargetPose() {
    Pose2d currentPose =
        RobotState.isAllianceRed()
            ? FlippingUtil.flipFieldPose(RobotState.getInstance().getEstimatedPose())
            : RobotState.getInstance().getEstimatedPose();
    double middle = DriveConstants.BLUE_HUB_ORIGIN.getY();
    Pose2d flippedUnderTrench =
        new Pose2d(
            DriveConstants.BLUE_UNDER_TRENCH_DEFENSE_POSE.getX(),
            middle + middle - DriveConstants.BLUE_UNDER_TRENCH_DEFENSE_POSE.getY(),
            new Rotation2d());
    ArrayList<Pose2d> poses = new ArrayList<>();
    poses.add(DriveConstants.BLUE_UNDER_TRENCH_DEFENSE_POSE);
    poses.add(flippedUnderTrench);
    poses.add(DriveConstants.BLUE_LEFT_CLIMB_DEFENSE_POSE);
    poses.add(DriveConstants.BLUE_RIGHT_CLIMB_DEFENSE_POSE);
    Pose2d closestPose = poses.get(0);
    for (Pose2d pose : poses) {
      if (closestPose.getTranslation().getDistance(currentPose.getTranslation())
          > pose.getTranslation().getDistance(currentPose.getTranslation())) {
        closestPose = pose;
      }
    }
    double angle =
        Math.atan2(
            closestPose.getY() - middle,
            closestPose.getX() - DriveConstants.BLUE_HUB_ORIGIN.getX());
    return new Pose2d(
        closestPose.getX(), closestPose.getY(), new Rotation2d(angle).plus(Rotation2d.kPi));
  }
}
