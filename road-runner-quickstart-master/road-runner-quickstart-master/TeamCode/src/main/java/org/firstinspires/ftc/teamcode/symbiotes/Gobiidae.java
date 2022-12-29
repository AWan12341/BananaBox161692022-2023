package org.firstinspires.ftc.teamcode.symbiotes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Date;

public class Gobiidae {

    FreightFrenzyPipeline pipeline; // Init the pipeline itself

    public static volatile int avg1, avg2, workingThres; // Create our avgs so they can be accessed

    public static volatile int height = 0; // What height do we compute?

    // This determines if we can see the right most piece or not
    // If your camera or auto is set up so you can see the left, but not the right,
    // This should be changed to reflect that.
    public static boolean canSeeRight = true;

    // Create a new date object to save the timestamp of calibration.
    Date date = new Date();

    //Do setup for the cameras and pipeline
    public void setup(HardwareMap hardwaremapInput, boolean dummy) {
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwaremapInput.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwaremapInput.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwaremapInput.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        pipeline = new FreightFrenzyPipeline();

        webcam.setPipeline(pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public static class FreightFrenzyPipeline extends OpenCvPipeline {


        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * If you want to enlarge or reduce the size of the regions, it should be done here.
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200, 110);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(40, 110);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and  work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        public int avgComplete, max, min;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));

//            workingThres = storage.STORED_THRESHOLD; // Call up our Threshold and save it from storage

        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because the ducks, and often team shipping elements,
             * will be a brighter color, such as yellow, and contrast strongly.
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over area. After this, we find which has the highest
             * concentration of color. That will be our element.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the element.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 tape markers, and
             * be small enough such that only the duck or team shipping element is sampled,
             * and not any of the surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            min = Math.min(avg1, avg1);
            avgComplete = (avg1 + avg2) / 2; // Avg of both values
            max = Math.max(avg1, avg2); // Max value



            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);


            /*
             * To compute the height, we essentially check if
             * one of the averages added to the threshold is less than
             * the second average.
             * This works fairly well in testing, but needs testing in more environments.
             */
            if(avg1 + workingThres < avg2){
                if(canSeeRight)
                    height = 3;
                else
                    height = 1;

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            }
            else if (avg2 + workingThres < avg1){
                height = 2;

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            }
            else {
                if(canSeeRight)
                    height = 1;
                else
                    height = 3;
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }


    }


    /*
     *Run this in ONLY our calibration opmode!
     * This program will calculate the difference between the highest and lowest region it can see.
     * By returning this, we are able to know what we are looking for as a threshold.
     */
//    public int calibrateRoutine(){
//
//        //Calculate the difference between the lowest value and the average of all values
//        int calc = (Math.max(avg1, avg2) - ((avg1 + avg2)/2));
//
//        //Store
//        storage.STORED_THRESHOLD = calc;
//        storage.TIMESTAMP = date.toString();
//        //If using Json to store your threshold, it should be done here.
//
//        //return val
//        return calc;
//
//    }
//
//    public String getCalibrationTimestamp(){
//        return storage.TIMESTAMP;
//    }
//
//    public int getStoredThreshold() { return storage.STORED_THRESHOLD; }

    //Return our height
    public int getAnalysis(){
        return height;
    }

    public int getAvg1(){
        return avg1;
    }

    public int getAvg2(){
        return avg2;
    }


}