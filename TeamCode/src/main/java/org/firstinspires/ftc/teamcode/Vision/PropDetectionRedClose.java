package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Disabled
@Config
public class PropDetectionRedClose implements VisionProcessor {

    public int detection = 2;

    public static int rightRectX1 = 30, rightRectY1 = 225;
    public static int rightRectX2 = 110, rightRectY2 = 350;

    public static double rightThresh = 1500000;
    public double rightSum = 0;

    public static int middleRectX1 = 340, middleRectY1 = 230;
    public static int middleRectX2 = 430, middleRectY2 = 330;

    public static double middleThresh = 900000;
    public double middleSum = 0;

    public static int redLowH = 100, redLowS = 40, redLowV = 0;
    public static int redHighH = 120, redHighS = 180, redHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);

        Rect rightRect = new Rect(new Point(rightRectX1, rightRectY1), new Point(rightRectX2, rightRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(redLowH, redLowS, redLowV);
        Scalar highThresh = new Scalar(redHighH, redHighS, redHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(rightSum > rightThresh)
            detection = 3;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 1;

//        workingMat.copyTo(frame);

        workingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}