package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;
@Autonomous
public class AutoRedStage extends LinearOpMode {
    // Init vision
    private VisionPortal visionPortal;
    private PropVision propVision = new PropVision(this.telemetry,true);

    private Robot robot = new Robot(true);

    private enum State{
        SPIKEMARK,
        WAIT1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        IDLE
    }
    private State currentState = State.SPIKEMARK;

    // Define a starting position
    Pose2d startPose = AutoConstants.RED_RIGHT_START;
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
        robot.intake.setHeight(robot.intake.UP);

        TrajectorySequence placeLeft = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToSplineHeading(AutoConstants.RED_RIGHT_LEFT_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.RED_LEFT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .waitSeconds(1)
                .strafeRight(22)
                .lineToSplineHeading(AutoConstants.RED_PARK)
                .build();

        TrajectorySequence placeCenter = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .lineToSplineHeading(AutoConstants.RED_RIGHT_CENTER_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .lineToSplineHeading(AutoConstants.RED_CENTER_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .strafeRight(13)
                .lineToSplineHeading(AutoConstants.RED_PARK)
                .build();

        TrajectorySequence placeRight = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToSplineHeading(AutoConstants.RED_RIGHT_RIGHT_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .strafeRight(6)
                .lineToSplineHeading(AutoConstants.RED_RIGHT_STAGE)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .strafeRight(5)
                .lineToSplineHeading(AutoConstants.RED_PARK)
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
        visionPortal.close();

        if (isStopRequested()) return;
        // Start has been pressed


        if(location == PropVision.PropLocation.LEFT){
            drive.followTrajectorySequenceAsync(placeLeft);
        } else if (location == PropVision.PropLocation.CENTER){
            drive.followTrajectorySequenceAsync(placeCenter);
        } else {
            drive.followTrajectorySequenceAsync(placeRight);

        }

        robot.arm.fullLock();
        robot.arm.ground();

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
