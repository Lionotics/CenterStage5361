package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/*
Links I am saving for reference:

https://github.com/OpenFTC/EasyOpenCV/tree/master
https://ftc-code.gitbook.io/tech-toolbox/computer-vision/color-thresholding

Example blob detectors:
https://github.com/yuhsb-lionotics/FreightFrenzyLight/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpenCvDetector.java

https://github.com/trc492/FtcTemplate/blob/master/TeamCode/src/main/java/teamcode/vision/Vision.java
https://github.com/Blockheads-5921/5921-Blockheads-PowerPlay/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/BasicPipeline.java

*/


public class PropVision implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat mat = new Mat();
        // Convert to HSB
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        // TODO: Tuning
        Scalar lowHSV = new Scalar(160, 50, 50); // lower bound HSV for red
        Scalar highHSV = new Scalar(180, 255, 255); // higher bound HSV for red

        Mat thresh = new Mat();
        Core.inRange(mat,lowHSV,highHSV,thresh);
        thresh.copyTo(frame);
        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
