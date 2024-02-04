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
@Autonomous
public class AutoRedBackstage extends LinearOpMode {
    // Init vision
    private VisionPortal visionPortal;
    private PropVision propVision = new PropVision(this.telemetry,true);

    private Robot robot = new Robot(true);

    private enum State{
        SPIKEMARK,
        IDLE
    }
    private State currentState = State.SPIKEMARK;

    // Define a starting position
    Pose2d startPose = AutoConstants.RED_BACKSTAGE_START;
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
        robot.arm.down();
        robot.slides.setTarget(0);

        TrajectorySequence placeLeft = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .addTemporalMarker(()->{
                    robot.slides.setTarget(0);
                })
                .forward(22)
                .lineToSplineHeading(AutoConstants.RED_LEFT_LEFT_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                })
                .lineToSplineHeading(AutoConstants.RED_LEFT_LEFT_EXTRA_MIDPOINT)
                .forward(20)
                .lineToSplineHeading(AutoConstants.RED_LEFT_MIDPOINT)
                .waitSeconds(9)
                .forward(50)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .splineTo(new Vector2d(AutoConstants.RED_LEFT_STAGE.getX()+4.25,AutoConstants.RED_LEFT_STAGE.getY()),AutoConstants.RED_LEFT_STAGE.getHeading())
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .back(7, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence placeCenter = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToSplineHeading(AutoConstants.RED_LEFT_CENTER_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                })
                .strafeLeft(13)
                .lineToSplineHeading(AutoConstants.RED_LEFT_MIDPOINT)
                .waitSeconds(9)
                .forward(50)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .splineTo(new Vector2d(AutoConstants.RED_CENTER_STAGE.getX() + 4.25,AutoConstants.RED_CENTER_STAGE.getY()),AutoConstants.RED_CENTER_STAGE.getHeading())
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.upMore();
                })
                .back(2, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .addTemporalMarker(()->{
                    robot.arm.down();
                })
                .back(5)
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })


                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence placeRight = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToSplineHeading(AutoConstants.RED_LEFT_RIGHT_SPIKEMARK)
                .addTemporalMarker(()->{
                    robot.arm.release2();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    robot.arm.down();
                })
                .strafeLeft(6)
                .lineToSplineHeading(AutoConstants.RED_LEFT_MIDPOINT)
                .waitSeconds(9)
                .forward(55)
                .addTemporalMarker(()->{
                    robot.arm.up();
                    robot.slides.setTarget(Slides.SLIDES_AUTO);
                })
                .splineTo(new Vector2d(AutoConstants.RED_RIGHT_STAGE.getX() + 4.25,AutoConstants.RED_RIGHT_STAGE.getY()),AutoConstants.RED_RIGHT_STAGE.getHeading())
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    robot.arm.release1();
                })
                .waitSeconds(1)
                .back(7, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(10))
                .addTemporalMarker(()->{
                    robot.arm.down();
                    robot.arm.fullRelease();
                    robot.slides.setTarget(0);
                })
                .turn(Math.toRadians(90))
                .build();

        // init loop. Runs during init before start is pressed
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


        if(location == PropVision.PropLocation.RIGHT){
            drive.followTrajectorySequenceAsync(placeRight);
        } else if (location == PropVision.PropLocation.CENTER){
            drive.followTrajectorySequenceAsync(placeCenter);
        } else {
            drive.followTrajectorySequenceAsync(placeLeft);

        }

        robot.arm.fullLock();
        robot.arm.ground();

        while(opModeIsActive() && !isStopRequested()){

            switch (currentState){
                case SPIKEMARK:
                    if(!drive.isBusy()){
                        currentState = State.IDLE;
                    }
                case IDLE:
                    break;
            }
            // UPDATE EVERYTHING
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
