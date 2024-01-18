package org.firstinspires.ftc.teamcode.vision.apriltag;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.List;

public class Apriltag implements VisionProcessor {
    private AprilTagProcessor aprilTag;

    // Making our own list of where the apriltags, in ways we can process more easily.
    // We lose some accuracy this way, but it's a start hopefully


    public static final Pose2d[] APRILTAG_POSES = new Pose2d[]{
            new Pose2d(60.25, 41.41, Math.toRadians(0)), // id 1
            new Pose2d(60.25, 35.41, Math.toRadians(0)), // id 2
            new Pose2d(60.25, 29.41, Math.toRadians(0)), // id 3
            new Pose2d(60.25, -29.41, Math.toRadians(0)), // id 4
            new Pose2d(60.25, -35.41, Math.toRadians(0)), // id 5
            new Pose2d(60.25, -41.41, Math.toRadians(0)), // id 6
            new Pose2d(-70.25, -40.625, Math.toRadians(180)), // id 7
            new Pose2d(-70.25,-35.125,Math.toRadians(180)), // id 8
            new Pose2d(-70.25,35.125,Math.toRadians(180)), // id 9
            new Pose2d(-70.25,40.625,Math.toRadians(180)), // id 10
    };
    public Pose2d calculatePose(int tagId, double x, double y, double heading){
        Pose2d relative = new Pose2d(x,y,heading);
        Pose2d absolute = APRILTAG_POSES[tagId - 1].minus(relative);
        return absolute;
    }
    List<AprilTagDetection> currentDetections;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        aprilTag.processFrame(frame,captureTimeNanos);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        ArrayList<AprilTagDetection> tags;

//        for(ApriltagDetection detection : currentDetections){
//            if(detection.metadata != null){
//
//
//            }
//        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
