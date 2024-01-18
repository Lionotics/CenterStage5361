package org.firstinspires.ftc.teamcode.opmodes.testing;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.vision.apriltag.Apriltag;
import org.firstinspires.ftc.teamcode.vision.prop.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagPose;

import java.util.List;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Apriltag testing", group = "testers")
@Config


public class ApriltagTesting extends LinearOpMode {

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    // Prop Vision
    private PropVision propVision = new PropVision(this.telemetry,false);



    private Pose2d robot = new Pose2d(0,0,0);

    
    double x, y, heading, headingRadians;

    boolean singleId = false;

    int singleIdNumber = 5;  // only used if singleId is true

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                processAprilTags();

                // Push telemetry to the Driver Station.

                TelemetryPacket packet = new TelemetryPacket();
                DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(0,0,0));
                packet.fieldOverlay().strokeCircle(0,0,5);
                dashboard.sendTelemetryPacket(packet);
                telemetry.update();


                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                if (gamepad1.dpad_left) {
                    singleId = true;
                } else if (gamepad1.dpad_right) {
                    singleId = false;
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()



    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                // == CAMERA CALIBRATION ==
                // TODO: Check camera calibration things
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

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

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void initPropVision(){
    }

    public void processAprilTags(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagPose> tags;

        for (AprilTagDetection detection: currentDetections){
            if( detection.metadata != null && (!singleId || detection.id == singleIdNumber) ){

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine("The position of that apriltag on the field is : " + detection.metadata.fieldPosition.getData());
                telemetry.addLine("The orientation of that apriltag on the field is : " + detection.metadata.fieldOrientation.toString());

                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");

                heading =  Math.toDegrees(Apriltag.APRILTAG_POSES[detection.id - 1].getHeading()) - detection.ftcPose.yaw;

                headingRadians = Math.toRadians(heading);

//                // The following is based on code I found on the internet
//                // Start by doing the rotation
//                double x2 = detection.ftcPose.x*Math.cos(headingRadians)+detection.ftcPose.y*Math.sin(headingRadians);
//                double y2 = detection.ftcPose.x*Math.sin(headingRadians)*(-1)+detection.ftcPose.y*Math.cos(headingRadians);
//                // then use those to get the absolute
//                double absoluteX = Apriltag.APRILTAG_POSES[detection.id - 1].getX() + y2;
//                double absoluteY = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - x2;

                x = Apriltag.APRILTAG_POSES[detection.id - 1].getX() - (detection.ftcPose.y * Math.cos(headingRadians) + (detection.ftcPose.x * Math.sin(headingRadians)));
                y = Apriltag.APRILTAG_POSES[detection.id - 1].getY() - (detection.ftcPose.y * Math.sin(headingRadians) - (detection.ftcPose.x * Math.cos(headingRadians)));

//                telemetry.addData("x*sin + y*cos", detection.ftcPose.x*Math.sin(heading)+detection.ftcPose.y*Math.cos(heading));
//                telemetry.addData("x*sin - y*cos", detection.ftcPose.x*Math.sin(heading)-detection.ftcPose.y*Math.cos(heading));
//                telemetry.addData("-x*sin + y*cos", -detection.ftcPose.x*Math.sin(heading)+detection.ftcPose.y*Math.cos(heading));
//                telemetry.addData("-x*sin - y*cos", -detection.ftcPose.x*Math.sin(heading)-detection.ftcPose.y*Math.cos(heading));
//                telemetry.addData("y*sin + x*cos", detection.ftcPose.y*Math.sin(heading)+detection.ftcPose.x*Math.cos(heading));
//                telemetry.addData("y*sin - x*cos", detection.ftcPose.y*Math.sin(heading)-detection.ftcPose.x*Math.cos(heading));
//                telemetry.addData("-y*sin + x*cos", -detection.ftcPose.y*Math.sin(heading)+detection.ftcPose.x*Math.cos(heading));
//                telemetry.addData("-y*sin - x*cos", -detection.ftcPose.y*Math.sin(heading)-detection.ftcPose.x*Math.cos(heading));
                telemetry.addData("relative x", detection.ftcPose.x);
                telemetry.addData("relative y", detection.ftcPose.y);
                telemetry.addData("relative heading", detection.ftcPose.yaw);
                telemetry.addData("tag x",Apriltag.APRILTAG_POSES[detection.id - 1].getX());
                telemetry.addData("tag y",Apriltag.APRILTAG_POSES[detection.id - 1].getY());
                telemetry.addData("x", x);
                telemetry.addData("y", y);
//                telemetry.addData("new absolute x",absoluteX);
//                telemetry.addData("new absolute y",absoluteY);
                telemetry.addData("heading", heading);

            }
        }



    }

}   // end class
