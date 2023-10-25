package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.PropVision;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Red Prop Vision Test")
public class PropVisionTesting extends LinearOpMode {

    private VisionPortal visionPortal;
    private PropVision propVision = new PropVision(this.telemetry,true);

    @Override
    public void runOpMode() throws InterruptedException {
        initPropVision();
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){

                telemetry.update();
                sleep(20);

            }

        }

        visionPortal.close();

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
