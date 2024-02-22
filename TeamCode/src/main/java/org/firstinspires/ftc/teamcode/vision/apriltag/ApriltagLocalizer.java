package org.firstinspires.ftc.teamcode.vision.apriltag;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ApriltagLocalizer implements Localizer {

    private AprilTagProcessor aprilTag;
    private HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private double finalX, finalY,heading, headingRadians;



    public ApriltagLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, VisionPortal visionPortal){

        this.hardwareMap = hardwareMap;
        this.visionPortal = visionPortal;
        initAprilTag();
        enableApriltagProcessor();

    }
    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


    }

    public void enableApriltagProcessor(){
        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableApriltagProcessor(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(finalX, finalY,heading);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(0,0,0);
    }

    @Override
    public void update() {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagPose> tags;

        for (AprilTagDetection detection: currentDetections){
            if(detection.metadata != null){
                // TODO: Filtering options for the apriltags
                heading =  Math.toDegrees(Apriltag.APRILTAG_POSES[detection.id - 1].getHeading()) - detection.ftcPose.yaw;
                headingRadians = Math.toRadians(heading);
                finalX = Apriltag.APRILTAG_POSES[detection.id - 1].getX() - (detection.ftcPose.y * Math.cos(headingRadians) + (detection.ftcPose.x * Math.sin(headingRadians)));
                finalY = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - (detection.ftcPose.y * Math.sin(headingRadians) - (detection.ftcPose.x * Math.cos(headingRadians)));

            }
        }

        // end update function
    }

    // update, but with the IMU heading
    public void update(double IMUHeading) {
        // TODO: Test this function
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagPose> tags;

        for (AprilTagDetection detection: currentDetections){
            if(detection.metadata != null){
                // TODO: Filtering options for the apriltags
                heading =  IMUHeading;
                headingRadians = Math.toRadians(heading);
                finalX = Apriltag.APRILTAG_POSES[detection.id - 1].getX() - (detection.ftcPose.y * Math.cos(headingRadians) + (detection.ftcPose.x * Math.sin(headingRadians)));
                finalY = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - (detection.ftcPose.y * Math.sin(headingRadians) - (detection.ftcPose.x * Math.cos(headingRadians)));

            }
        }
    }
}

