package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Drive.SwerveSubsystem;

public class AlignToGoalCommand extends Command{
    private Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;
    public AlignToGoalCommand (Vision vision) {
        this.poseSupplier = () -> vision.getPosition(); //replace with function for getting position
        addRequirements(vision);
    }
    public AlignToGoalCommand (Robotstate robotState) {
        this.poseSupplier = () -> robotState.getRobotPoseFromSwerveDriveOdometry();
    }

    private Pose2d getTargetPose() {
        targetPose = poseSupplier.get();
        
    }
}
