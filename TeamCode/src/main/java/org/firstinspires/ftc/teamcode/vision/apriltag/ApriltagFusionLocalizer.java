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
public class ApriltagFusionLocalizer implements Localizer {

    private final TwoWheelTrackingLocalizer deadWheelLocalizer;
    private AprilTagProcessor aprilTag;
    private final HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private double finalX;
    private double finalY;
    private double finalHeading;
    // these need defining and tuning probably
    private double odometryVarianceX = 1;
    private double odometryVarianceY = 1;
    private double odometryVarianceHeading = 1;


    public ApriltagFusionLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, VisionPortal visionPortal){

        deadWheelLocalizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);
        this.hardwareMap = hardwareMap;
        this.visionPortal = visionPortal;
        initAprilTag();
        enableApriltagProcessor();
        finalX = 0;
        finalY = 0;


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
        return new Pose2d(finalX, finalY,finalHeading);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        deadWheelLocalizer.setPoseEstimate(pose2d);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return deadWheelLocalizer.getPoseVelocity();
    }

    @Override
    public void update() {
        // Update dead wheel estimate
        deadWheelLocalizer.update();
        // Change the variance of the odometry based on how far we've gone -- movement variance
        // I'm not sure what the best way to calculate the total distance on the odometry - encoder ticks go up and down as the robot drives around the field
        // TODO: Put code here
        double odometryX = deadWheelLocalizer.getPoseEstimate().getX();
        double odometryY = deadWheelLocalizer.getPoseEstimate().getY();
        double odometryHeading = deadWheelLocalizer.getHeading();

        double ODOMETRY_ERROR_CONSTANT = 0.01;
        odometryVarianceX += ODOMETRY_ERROR_CONSTANT * deadWheelLocalizer.getPoseVelocity().getX();
        odometryVarianceY += ODOMETRY_ERROR_CONSTANT * deadWheelLocalizer.getPoseVelocity().getY();
        odometryVarianceHeading += ODOMETRY_ERROR_CONSTANT * deadWheelLocalizer.getPoseVelocity().getHeading();

        // if no apriltags were found, just use the dead wheel estimate
        finalX = deadWheelLocalizer.getPoseEstimate().getX();
        finalY = deadWheelLocalizer.getPoseEstimate().getY();
        finalHeading = deadWheelLocalizer.getPoseEstimate().getHeading();


        // Process the apriltags and turn them into positions
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagPose> tags;

        //TODO: Check for closest rather than using the first
        for (AprilTagDetection detection: currentDetections) {
            if (detection.metadata != null) {
                double apriltagHeading = Math.toDegrees(Apriltag.APRILTAG_POSES[detection.id - 1].getHeading()) - detection.ftcPose.yaw;
                double CAMERA_OFFSET_X = -0.25;
                double apriltagX = Apriltag.APRILTAG_POSES[detection.id - 1].getX() - (detection.ftcPose.y * Math.cos(Math.toRadians(apriltagHeading)) + (detection.ftcPose.x * Math.sin(Math.toRadians(apriltagHeading)))) + CAMERA_OFFSET_X;
                double CAMERA_OFFSET_Y = -6.19;
                double apriltagY = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - (detection.ftcPose.y * Math.sin(Math.toRadians(apriltagHeading)) - (detection.ftcPose.x * Math.cos(Math.toRadians(apriltagHeading)))) + CAMERA_OFFSET_Y;
                // tag_var_x = k1x^2 + k2x + k3     same for y
                // to do: replace with binomial
                double APRILTAG_ERROR_CONSTANT = 1;
                double apriltagVarianceX = detection.ftcPose.range * APRILTAG_ERROR_CONSTANT;

                double apriltagVarianceY = detection.ftcPose.range * APRILTAG_ERROR_CONSTANT;

                double apriltagVarianceHeading = detection.ftcPose.range * APRILTAG_ERROR_CONSTANT;


                // Set the main estimate with weighted average
                finalX = (odometryX * apriltagVarianceX + apriltagX * odometryVarianceX) / (apriltagVarianceX + odometryVarianceX);
                finalY = (odometryY * apriltagVarianceY + apriltagY * odometryVarianceY) / (apriltagVarianceY + odometryVarianceY);
                finalHeading = (odometryHeading * apriltagVarianceHeading + apriltagHeading * odometryVarianceHeading) / (apriltagVarianceHeading + odometryVarianceHeading);

                Pose2d estimatedPose = new Pose2d(finalX, finalY, finalHeading);
                setPoseEstimate(estimatedPose);

                double finalVarianceX = (odometryVarianceX * apriltagVarianceX) / (odometryVarianceX + apriltagVarianceX);
                double finalVarianceY = (odometryVarianceY * apriltagVarianceY) / (odometryVarianceY + apriltagVarianceY);
                // this changes over time in some form
                double finalVarianceHeading = (odometryVarianceHeading * apriltagVarianceHeading) / (odometryVarianceHeading + apriltagVarianceHeading);

                odometryVarianceX = finalVarianceX;
                odometryVarianceY = finalVarianceY;
                odometryVarianceHeading = finalVarianceHeading;

                break;
            }
        }


    // end update function
    }

    public ArrayList<Double> getATVariances(double x, double y, double heading) {
        double varX=0.0, varY=0.0, varH=0.0;
        // TODO: Actual math goto:140
        varX = 0;
        varY = 0;
        varH = 0;
        return new ArrayList<Double>(Arrays.asList(varX, varY, varH));
    }

}

