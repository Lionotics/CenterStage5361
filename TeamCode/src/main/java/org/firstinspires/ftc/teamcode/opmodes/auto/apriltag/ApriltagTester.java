package org.firstinspires.ftc.teamcode.opmodes.auto.apriltag;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.apriltag.ApriltagLocalizer;
import org.firstinspires.ftc.teamcode.vision.prop.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "ApriltagTester")
public class ApriltagTester extends LinearOpMode {
    private VisionPortal visionPortal;
    // Vision set to RED
//    private PropVision propVision = new PropVision(this.telemetry,true);
    private ApriltagLocalizer apriltagLocalizer = new ApriltagLocalizer();
    private Pose2d apriltagPose = new Pose2d(0,0,0);

//    private ApriltagLocalizer apriltagLocalizer = new ApriltagLocalizer(visionPortal);


    PropVision.PropLocation location;
    @Override
    public void runOpMode() throws InterruptedException {

        // setup vision
        initVision();
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive() && !isStopRequested()){
            apriltagPose = apriltagLocalizer.getRobotPose();

            if(apriltagPose.getX() !=0 && apriltagPose.getY() != 0){
                // if it's something real
                telemetry.addData("Apriltag Pose", apriltagPose);
            } else {
                telemetry.addData("Apriltag Pose", "No Pose");
            }
            telemetry.update();
            // Update any other things that need updating every loop here too (e.g slides)
            sleep(250);
        }
    }

    private void initVision(){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessors(apriltagLocalizer);
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(apriltagLocalizer, true);
    }
}
