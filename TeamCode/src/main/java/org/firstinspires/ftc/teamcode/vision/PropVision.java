package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
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

    // bounding lines for the three regions
    int height = 480;
    int width = 640;

    int LEFTLINE = (int)(width / 3);
    int RIGHTLINE = (int)(2 * (width / 3));

    public enum PropLocation{
        LEFT,
        CENTER,
        RIGHT
    }
    private PropLocation location;
    // set our color
    boolean isRed = true;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    Telemetry telemetry;

    public PropVision(Telemetry telemetry, boolean isRed){
        this.telemetry = telemetry;
        this.isRed = isRed;

    }
    // Constructor for EOCV-sim
    public PropVision(Telemetry telemetry){
        this.telemetry = telemetry;
        this.isRed = true;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Scalar lowHSV = new Scalar(0,0,0);
        Scalar highHSV = new Scalar(255,255,255);

        // TODO: Tuning for both red and blue
        // add an if statement based on isRed
        if(isRed) {
            lowHSV = new Scalar(160, 50, 50); // lower bound HSV for red
            highHSV = new Scalar(180, 255, 255); // higher bound HSV for red
        } else {
            // Insert blue values here
            lowHSV = new Scalar(110, 50, 50); // lower bound HSV for red
            highHSV = new Scalar(120, 255, 255); // higher bound HSV for red

        }
        Mat mat = new Mat();
        // Convert to HSv
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

        // TODO: Tuning
        Mat thresh = new Mat();
        Core.inRange(mat,lowHSV,highHSV,thresh);
        Mat left = thresh.submat(0,height,0,LEFTLINE);
        Mat center = thresh.submat(0,height,LEFTLINE,RIGHTLINE);
        Mat right = thresh.submat(0,height,RIGHTLINE,width);
        // draw lines to make it clear where the divide is
        Imgproc.line(frame,new Point(LEFTLINE,0), new Point(LEFTLINE,height),GREEN,5);
        Imgproc.line(frame,new Point(RIGHTLINE,0), new Point(RIGHTLINE,height),GREEN,5);


        //Calculate number of "trues" in each, and select the one with the most
        int leftNum = Core.countNonZero(left);
        int centerNum = Core.countNonZero(center);
        int rightNum = Core.countNonZero(right);

        if (leftNum > centerNum && leftNum > rightNum){
            location = PropLocation.LEFT;
        } else if (centerNum > leftNum && centerNum > rightNum){
            location = PropLocation.CENTER;
        } else if (rightNum > centerNum && rightNum > leftNum){
            location = PropLocation.RIGHT;
        }

        telemetry.addLine("LEFT: " + leftNum);
        telemetry.addLine("CENTER: " + centerNum);
        telemetry.addLine("RIGHT: " + rightNum);
        telemetry.addLine("Location: " + location);
        telemetry.update();

        // our camera output gets put back into the frame - showing which pixels are being used
        frame.copyTo(frame);
        // be responsible with memory
        mat.release();
        thresh.release();
        left.release();
        right.release();
        center.release();

        return null;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
