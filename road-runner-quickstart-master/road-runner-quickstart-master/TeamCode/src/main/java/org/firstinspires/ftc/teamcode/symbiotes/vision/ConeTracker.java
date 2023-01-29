package org.firstinspires.ftc.teamcode.symbiotes.vision;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class ConeTracker extends OpenCvPipeline {

    //Determine the color we want to detect.
    //Todo: Make this change depending on what side of the field we're on
    public DETECT_COLOR color = DETECT_COLOR.BOTH;
    public double CONTOUR_AREA = 250.0;
    private final Scalar BOUNDING_COLOR = new Scalar(255, 0, 255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    public Scalar TEXT_COLOR = new Scalar(0, 0, 0);

    private Rect redRect;
    private Rect blueRect;

    public double horizon = 130;

    private final List<MatOfPoint> redContours;
    private final List<MatOfPoint> blueContours;

    private MatOfPoint biggestRedContour;
    private MatOfPoint biggestBlueContour;

    enum DETECT_COLOR {
        RED,
        BLUE,
        BOTH
    }

    public ConeTracker() {
        redContours = new ArrayList<>();
        redRect = new Rect();
        biggestRedContour = new MatOfPoint();

        blueContours = new ArrayList<>();
        blueRect = new Rect();
        biggestBlueContour = new MatOfPoint();
    }

    // Red masking thresholding values
    public Scalar lowRed = new Scalar(0, 161, 60);
    public Scalar highRed = new Scalar(200, 255, 255);

    // Blue masking thresholding values
    public Scalar lowBlue = new Scalar(0, 80, 138);
    public Scalar highBlue = new Scalar(100, 255, 255);

    // Mat objects
    private final Mat maskRed = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat YCrCb = new Mat();

    // Kernel size for blurring
    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);

    @Override
    public Mat processFrame(Mat input) {
        //Convert RGB space to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        //Do erosion for better detection
        Imgproc.erode(YCrCb, YCrCb, kernel);

        //For red cones
        if (color.equals(DETECT_COLOR.RED) || color.equals(DETECT_COLOR.BOTH)) {
            //Ensure the cone is within the acceptable color range
            inRange(YCrCb, lowRed, highRed, maskRed);

            redContours.clear();
            //Do contour detection
            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //Remove the found object if it is above the horizon line
            redContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            //Draw the contours on
            Imgproc.drawContours(input, redContours, -1, BOUNDING_COLOR);
            //If we've found contours...
            if(!redContours.isEmpty()) {
                //Check if the contour we've found is the largest one
                biggestRedContour = Collections.max(redContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                //Check if the biggest contour we found is larger than a threshold
                if(Imgproc.contourArea(biggestRedContour) > CONTOUR_AREA) {
                    //Generate a boundary around the biggest contour
                    redRect = Imgproc.boundingRect(biggestRedContour);
                    //Set the color and type of the bounding rectangle
                    Imgproc.rectangle(input, redRect, BOUNDING_COLOR, 2);
                    //Place some text for identification
                    Imgproc.putText(input, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                }
            }

            maskRed.release();
        }

        //For Blue cones
        if (color.equals(DETECT_COLOR.BLUE) || color.equals(DETECT_COLOR.BOTH)) {
            inRange(YCrCb, lowBlue, highBlue, maskBlue);

            blueContours.clear();

            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            blueContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(input, blueContours, -1, BOUNDING_COLOR);

            if(!blueContours.isEmpty()) {
                biggestBlueContour = Collections.max(blueContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestBlueContour) > CONTOUR_AREA) {
                    blueRect = Imgproc.boundingRect(biggestBlueContour);

                    Imgproc.rectangle(input, blueRect, BOUNDING_COLOR, 2);
                    Imgproc.putText(input, "Blue Cone", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
//                    Imgproc.putText(input, ("X Pos: " + blueRect.x), new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                }
            }
            maskBlue.release();
        }

        //Generate the horizon line
        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        YCrCb.release();

        return input;
    }

    //Return the closest rectangle
    public Rect getClosestCone() {
        switch (color) {
            case RED:
                return redRect;
            case BLUE:
                return blueRect;
            case BOTH:
            default:
                return redRect.area() > blueRect.area() ? redRect : blueRect;
        }
    }

    public int getClosestConeCenter(){
        switch (color) {
            case RED:
                return redRect.x;
            case BLUE:
                return blueRect.x;
            case BOTH:
            default:
                return redRect.area() > blueRect.area() ? redRect.x : blueRect.x;
        }
    }
}