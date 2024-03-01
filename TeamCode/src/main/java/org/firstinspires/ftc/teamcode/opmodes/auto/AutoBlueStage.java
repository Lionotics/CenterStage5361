package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.prop.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous(name = "Auto: Blue Close 2+0")
public class AutoBlueStage extends LinearOpMode {
    // Init vision
    private VisionPortal visionPortal;
    // Vision set to BLUE
    private PropVision propVision = new PropVision(this.telemetry,false);

    private Robot robot = new Robot(true);

    private enum State{
        SPIKEMARK,
        IDLE
    }
    private State currentState = State.SPIKEMARK;

    // Define a starting position
    Pose2d startPose = AutoConstants.BLUE_FRONTSTAGE_START;
    // Vision
    PropVision.PropLocation location;
    @Override
    public void runOpMode() throws InterruptedException {

        // setup vision
        initPropVision();

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
                .splineTo(new Vector2d(AutoConstants.BLUE_LEFT_LEFT_SPIKEMARK.getX(),AutoConstants.BLUE_LEFT_LEFT_SPIKEMARK.getY()),AutoConstants.BLUE_LEFT_LEFT_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.BLUE_LEFT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(()->{
                    robot.slides.setTarget(Slides.SLIDES_AUTO+300);
                })
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getX(),AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
                .setReversed(false)
                .build();


        TrajectorySequence placeCenter = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(AutoConstants.BLUE_LEFT_CENTER_SPIKEMARK.getX(),AutoConstants.BLUE_LEFT_CENTER_SPIKEMARK.getY()),AutoConstants.BLUE_LEFT_CENTER_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.BLUE_CENTER_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(()->{
                    robot.slides.setTarget(Slides.SLIDES_AUTO+300);
                })
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getX(),AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
                .setReversed(false)
                .build();

        TrajectorySequence placeRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(AutoConstants.BLUE_LEFT_RIGHT_SPIKEMARK.getX(),AutoConstants.BLUE_LEFT_RIGHT_SPIKEMARK.getY()),AutoConstants.BLUE_LEFT_RIGHT_SPIKEMARK.getHeading())
                .back(7)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.BLUE_RIGHT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(0.25)
                .addTemporalMarker(()->{
                    robot.slides.setTarget(Slides.SLIDES_AUTO+300);
                })
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .setReversed(true)
                .splineTo(new Vector2d(AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getX(),AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getY()),
                        AutoConstants.BLUE_PARK_EDGE_WAYPOINT.getHeading()+Math.toRadians(180))
                .setReversed(false)
                .build();

        // init loop. Runs during init before start is pressed
        while(!isStarted() && !isStopRequested()){

            location = propVision.getLocation();
            telemetry.addData("Prop Location", location);
            telemetry.update();

        }

        location = propVision.getLocation();

        if(location == null){
            location = PropVision.PropLocation.RIGHT;
        }

        telemetry.addData("Selected Location", location);
        telemetry.update();


        visionPortal.setProcessorEnabled(propVision, false);

        if (isStopRequested()) return;
        // Start has been pressed


        if(location == PropVision.PropLocation.LEFT){
            drive.followTrajectorySequenceAsync(placeLeft);
        } else if (location == PropVision.PropLocation.CENTER){
            drive.followTrajectorySequenceAsync(placeCenter);
        } else {
            drive.followTrajectorySequenceAsync(placeRight);

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
            drive.update();
            robot.slides.pidLoop();
            // Update any other things that need updating every loop here too (e.g slides)

        }


    }

    private void initPropVision(){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(propVision);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(propVision, true);
    }
}
