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
import org.firstinspires.ftc.teamcode.vision.apriltag.Apriltag;
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

    private TwoWheelTrackingLocalizer deadWheelLocalizer;
    private AprilTagProcessor aprilTag;
    private HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private double apriltagX, apriltagY,heading,headingRadians,odometryX, odometryY, odometryHeading, finalX, finalY, finalHeading;
    // these need defining and tuning probably
    private double odometryVariance = 0.5; // this changes over time in some form
    private double apriltagVariance = 0.5;
    private double mainVariance = 1;

    public ApriltagLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, VisionPortal visionPortal){

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

            finalX = pose2d.getX();
            finalY = pose2d.getY();
            finalHeading = pose2d.getHeading();

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


        // Process the apriltags and turn them into positions
        // TODO: Figure out how the variance works here too
        // TODO: Multi-tag handling is broken and needs to be fixed
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagPose> tags;
        for (AprilTagDetection detection: currentDetections){
            if(detection.metadata != null){
                heading =  Math.toDegrees(Apriltag.APRILTAG_POSES[detection.id - 1].getHeading()) - detection.ftcPose.yaw;
                headingRadians = Math.toRadians(heading);
                apriltagX = Apriltag.APRILTAG_POSES[detection.id - 1].getX() - (detection.ftcPose.y * Math.cos(headingRadians) + (detection.ftcPose.x * Math.sin(headingRadians)));
                apriltagY = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - (detection.ftcPose.y * Math.sin(headingRadians) - (detection.ftcPose.x * Math.cos(headingRadians)));
                // TODO: compute april tag variances
                // tag_var_x = k1x^2 + k2x + k3     same for y

                odometryX = deadWheelLocalizer.getPoseEstimate().getX();
                odometryY = deadWheelLocalizer.getPoseEstimate().getY();
                odometryHeading = deadWheelLocalizer.getPoseEstimate().getHeading();
                // TODO: compute odo vaiances
                // odo_var_x = main_var_x + k1*velocity_x + k2*velocity_y
                // odo_var_x = main_var_y + k2*velocity_x + k1*velocity_y
                // var_heading = 0.015 (this should be with the constants)

                // Set the main estimate with weighted average
                finalX = (odometryX * apriltagVariance + apriltagX * odometryVariance) / (apriltagVariance + odometryVariance);
                finalY = (odometryY * apriltagVariance + apriltagY * odometryVariance) / (apriltagVariance + odometryVariance);
                finalHeading = (odometryHeading* apriltagVariance + heading * odometryVariance) / (apriltagVariance + odometryVariance);

                // How should I change the dead wheel estimate based on this new number? I'm not quite sure.
                // Something like this maybe?
                deadWheelLocalizer.setPoseEstimate(new Pose2d(finalX, finalY, finalHeading));

                // Set the main variance
                mainVariance = (apriltagVariance * odometryVariance)/(apriltagVariance + odometryVariance);



            }
        }

        // if no apriltags were found, just use the dead wheel estimate
        if(currentDetections.size() == 0){
            finalX = deadWheelLocalizer.getPoseEstimate().getX();
            finalY = deadWheelLocalizer.getPoseEstimate().getY();
            finalHeading = deadWheelLocalizer.getPoseEstimate().getHeading();
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

