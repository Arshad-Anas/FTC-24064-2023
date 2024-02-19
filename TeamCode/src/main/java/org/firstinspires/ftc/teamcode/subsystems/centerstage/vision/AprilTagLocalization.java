package org.firstinspires.ftc.teamcode.subsystems.centerstage.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class AprilTagLocalization {
    public static final double
            fx = 578.272,
            fy = 578.272,
            cx = 402.145,
            cy = 221.506;

    public final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    public AprilTagLocalization(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public List<AprilTagDetection> getRawDetections() {
        return aprilTagProcessor.getDetections();
    }

    public Pose2d getPoseEstimate() {
        Pose2d estimate = null;
        for (AprilTagDetection detection : getRawDetections()) {
            if (detection.metadata != null) {
                boolean isLargeTag = detection.metadata.id >= 7;
                int multiplier = isLargeTag ? 1 : -1;
                VectorF tagVec = detection.metadata.fieldPosition;
                estimate = new Pose2d(
                        tagVec.get(0) + (detection.ftcPose.y - DriveConstants.CAMERA_FORWARD_OFFSET /* For us: around -6 */) * multiplier,
                        tagVec.get(1) + (detection.ftcPose.x - DriveConstants.CAMERA_LATERAL_OFFSET /* Around 1 (?) */) * -multiplier,
                        Math.toRadians((isLargeTag ? 0 : 180) + detection.ftcPose.yaw)
                );
            }
        }

        return estimate;
    }
}
