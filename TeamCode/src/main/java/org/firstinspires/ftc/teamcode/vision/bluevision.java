package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import org.openftc.easyopencv.OpenCvPipeline;

public class bluevision extends OpenCvPipeline {

    public Scalar lowerRGB = new Scalar(0.0, 0.0, 201.0, 0.0);
    public Scalar upperRGB = new Scalar(255.0, 255.0, 255.0, 0.0);
    private Mat rgbMat = new Mat();
    private Mat rgbBinaryMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
    public int lineThickness = 3;

    private Mat inputContours = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_RGBA2RGB);
        Core.inRange(rgbMat, lowerRGB, upperRGB, rgbBinaryMat);

        contours.clear();
        hierarchy.release();
        Imgproc.findContours(rgbBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        input.copyTo(inputContours);
        Imgproc.drawContours(inputContours, contours, -1, lineColor, lineThickness);

        return inputContours;
    }
}