package org.firstinspires.ftc.teamcode.opmodes.auto.apriltag;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.apriltag.Apriltag;
import org.firstinspires.ftc.teamcode.vision.apriltag.ApriltagLocalizer;
import org.firstinspires.ftc.teamcode.vision.prop.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagPose;

import java.util.List;

@Autonomous(name = "ApriltagExperiment Red Close 2+0")
public class RedCloseApriltagExperiment extends LinearOpMode {
    // Init vision
    private VisionPortal visionPortal;
    // Vision set to RED
    private PropVision propVision = new PropVision(this.telemetry,true);
    private ApriltagLocalizer apriltagLocalizer = new ApriltagLocalizer();
    private Pose2d apriltagPose;

//    private ApriltagLocalizer apriltagLocalizer = new ApriltagLocalizer(visionPortal);

    private Robot robot = new Robot(true);

    private enum State{
        SPIKEMARK,
        IDLE
    }
    private State currentState = State.SPIKEMARK;

    // Define a starting position
    Pose2d startPose = AutoConstants.RED_FRONTSTAGE_START;
    // Vision
    PropVision.PropLocation location;
    @Override
    public void runOpMode() throws InterruptedException {

        // setup vision
        initVision();

        // setup roadrunner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        // setup other hardware
        robot.init(hardwareMap);
        robot.intake.intakeUp();
        robot.slides.setTarget(0);
        robot.arm.down();
        robot.arm.lock1();


        TrajectorySequence placeLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(AutoConstants.RED_RIGHT_LEFT_SPIKEMARK.getX(),AutoConstants.RED_RIGHT_LEFT_SPIKEMARK.getY()),AutoConstants.RED_RIGHT_LEFT_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.RED_LEFT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.RED_PARK_EDGE_WAYPOINT.getX(),AutoConstants.RED_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(AutoConstants.RED_PARK_EDGE.getX(),AutoConstants.RED_PARK_EDGE.getY()),
//                        AutoConstants.RED_PARK_EDGE.getHeading())
                .setReversed(false)
                .build();

        TrajectorySequence placeCenter = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(AutoConstants.RED_RIGHT_CENTER_SPIKEMARK.getX(),AutoConstants.RED_RIGHT_CENTER_SPIKEMARK.getY()),AutoConstants.RED_RIGHT_CENTER_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.RED_CENTER_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.RED_PARK_EDGE_WAYPOINT.getX(),AutoConstants.RED_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(AutoConstants.RED_PARK_EDGE.getX(),AutoConstants.RED_PARK_EDGE.getY()),
//                        AutoConstants.RED_PARK_EDGE.getHeading())
                .setReversed(false)
                .build();

        TrajectorySequence placeRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(AutoConstants.RED_RIGHT_RIGHT_SPIKEMARK.getX(),AutoConstants.RED_RIGHT_RIGHT_SPIKEMARK.getY()),AutoConstants.RED_RIGHT_RIGHT_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.RED_RIGHT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.RED_PARK_EDGE_WAYPOINT.getX(),AutoConstants.RED_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.RED_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(AutoConstants.RED_PARK_EDGE.getX(),AutoConstants.RED_PARK_EDGE.getY()),
//                        AutoConstants.RED_PARK_EDGE.getHeading())
                .setReversed(false)
                .build();

        // init loop. Runs durring init before start is pressed
        while(!isStarted() && !isStopRequested()){

            location = propVision.getLocation();
            telemetry.addData("Prop Location", location);
            telemetry.update();

        }

        location = propVision.getLocation();
        telemetry.addData("Selected Location", location);
        telemetry.update();
        // Stop all vision once opmode has started
        // (if we use apriltags this will need to be changed)
        visionPortal.setProcessorEnabled(propVision, false);
        // no liveview once auto has started
        visionPortal.stopLiveView();

        if (isStopRequested()) return;
        // Start has been pressed


        if(location == PropVision.PropLocation.RIGHT){
            drive.followTrajectorySequenceAsync(placeRight);
        } else if (location == PropVision.PropLocation.CENTER){
            drive.followTrajectorySequenceAsync(placeCenter);
        } else {
            drive.followTrajectorySequenceAsync(placeLeft);

        }



        while(opModeIsActive() && !isStopRequested()){

            switch (currentState){
                case SPIKEMARK:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
//                        robot.arm.release2();
                    }

                case IDLE:
                    break;
            }
            // UPDATE EVERYTHING WOW
            drive.update();
            robot.slides.pidLoop();

            apriltagPose = apriltagLocalizer.getRobotPose();

            if(!apriltagPose.equals(new Pose2d(0,0,0))){
                // if it's something real
                telemetry.addData("Apriltag Pose", apriltagPose);
            } else {
                telemetry.addData("Apriltag Pose", "No Pose");
            }
            telemetry.update();
            // Update any other things that need updating every loop here too (e.g slides)

        }


    }

    private void initVision(){


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessors(propVision,apriltagLocalizer);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(propVision, true);
        visionPortal.setProcessorEnabled(apriltagLocalizer, true);
    }


}
