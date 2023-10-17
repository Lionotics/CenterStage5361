package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.vision.PropVision;
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
@TeleOp(name = "Apriltag testing")
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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    
    double x, y, heading, headingRadians;
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
//                DashboardUtil.drawRobot(packet.fieldOverlay(), new Pose2d(0,0,0));
                packet.fieldOverlay().strokeCircle(0,0,5);
                dashboard.sendTelemetryPacket(packet);
                dashboardTelemetry.update();
                telemetry.update();


                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
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
            if(detection.metadata != null){

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine("The position of that apriltag on the field is : " + detection.metadata.fieldPosition.getData());
                telemetry.addLine("The orientation of that apriltag on the field is : " + detection.metadata.fieldOrientation.toString());

                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");

                heading = detection.metadata.fieldOrientation.z + detection.ftcPose.bearing;
                dashboardTelemetry.addData("FieldOrientationZ",detection.metadata.fieldOrientation.z);
                dashboardTelemetry.addData("FieldOrientationX",detection.metadata.fieldOrientation.x);
                dashboardTelemetry.addData("FieldOrientationY",detection.metadata.fieldOrientation.y);
                dashboardTelemetry.addData("ftcpose.yaw", detection.ftcPose.yaw);
                dashboardTelemetry.addData("ftcpose.pitch", detection.ftcPose.pitch);
                dashboardTelemetry.addData("ftcpose.roll", detection.ftcPose.roll);
                dashboardTelemetry.addData("bearing",detection.ftcPose.bearing);

                headingRadians = Math.toRadians(heading);
                x = detection.metadata.fieldPosition.get(0) - (detection.ftcPose.x * Math.cos(headingRadians) - (detection.ftcPose.y * Math.sin(headingRadians)));
                y = detection.metadata.fieldPosition.get(1) - (detection.ftcPose.x * Math.sin(headingRadians)) + (detection.ftcPose.y * Math.cos(headingRadians));
                robot = new Pose2d(x,y,heading);
                dashboardTelemetry.addData("x", robot.getX());
                dashboardTelemetry.addData("y", robot.getY());
                dashboardTelemetry.addData("heading", robot.getHeading());

                telemetry.addData("x", robot.getX());
                telemetry.addData("y", robot.getY());
                telemetry.addData("heading", robot.getHeading());



            }
        }



    }

}   // end class
