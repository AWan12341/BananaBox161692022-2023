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
import org.openftc.easyopencv.OpenCvWebcam;

public class ZitherArrayBasedDetection{

    //The var we will save the location of the item into
    public static volatile int savedLocation = 0;

    ZitherVisionPipeline pipeline; // Init the pipeline itself


    //This is the position where cells will be located, up and down the screen
    static final int yPosCells = 120;
    //How many cells we should create
    static final int cellCount = 50;
    static final int cellMidPoint = cellCount/2;
    //The X resolution of the camera
    static final int resolution = 320;

    static final int REGION_WIDTH = resolution/cellCount;
    static final int REGION_HEIGHT = 100;

    //Do setup for the cameras and pipeline
    public void setup(HardwareMap hardwaremapInput) {
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwaremapInput.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwaremapInput.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwaremapInput.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        pipeline = new ZitherVisionPipeline();

        webcam.setPipeline(pipeline);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public static class ZitherVisionPipeline extends OpenCvPipeline {


        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);


        /*
         * Working variables
         */
        Mat Cb_arr[] = new Mat[cellCount];
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();


        //The upper left anchor points of each cell
        Point CELLS_ARR_ANCHOR_POINTS[] = new Point[cellCount];

        /*
        Point A and B on an array. These describe the complete box.

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
         *   |                  Point B (x,y)   |
         *   ------------------------------------
         *
         */
        static final Point REGION_ARR_MOORING_POINTA[] = new Point[cellCount];
        static final Point REGION_ARR_MOORING_POINTB[] = new Point[cellCount];

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


            //Compile the Anchor Points
            for (int i = 0; i < cellCount; i++) {

                /*
                 * For every cell, step once through the anchor array, creating the new point
                 * as a function of the resolution and cell count.
                 * This is described as f(x) = (Xresolution/Amount_Of_cell) * step.
                 * If, as an example, we have a resolution of 320, are looking to create 14 cells,
                 * and we are putting an item into the 5th slot, the equation would be as follows:
                 * (320/14) * 5 which equals 114.28
                 * We also force this to be an integer so we don't end up with partial pixels
                 */

                CELLS_ARR_ANCHOR_POINTS[i] = new Point(((int) (resolution / cellCount) * i), yPosCells);
            }

            //Compile regions
            for (int i = 0; i < cellCount; i++) {
                //We simply set the A position equal to the anchor point of each cell
                //This will represent the upper left position of each cell
                REGION_ARR_MOORING_POINTA[i] = new Point(CELLS_ARR_ANCHOR_POINTS[i].x, CELLS_ARR_ANCHOR_POINTS[i].y);
            }

            //Compile regions
            for (int i = 0; i < cellCount; i++) {
                //We do the same thing for the Point B, but we add the
                //height and width to define the lower right most position
                REGION_ARR_MOORING_POINTB[i] = new Point(CELLS_ARR_ANCHOR_POINTS[i].x + REGION_WIDTH, CELLS_ARR_ANCHOR_POINTS[i].y + REGION_HEIGHT);
            }

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             * This function will compile the Submat array.
             */
            for (int i = 0; i < cellCount; i++) {

                Cb_arr[i] = Cb.submat(new Rect(REGION_ARR_MOORING_POINTA[i], REGION_ARR_MOORING_POINTB[i]));

            }
        }

        @Override
        public Mat processFrame(Mat input) {

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
            int averages[] = new int[cellCount];

            for (int i = 0; i < cellCount; i++) {
                averages[i] = (int) Core.mean(Cb_arr[i]).val[0];
            }

            /*
             * Find the min of the averages
             */

            //Set the min arbitrarily high, so it doesn't automatically
            //call it the minimum.
            int min = 500;
            for (int i = 0; i < cellCount; i++) {
                if (averages[i] < min) {
                    min = averages[i];
                    savedLocation = i;
                }
            }

            /*
             * Draw a rectangle onto the screen
             * This will be done for all rectangles
             * On the array
             * Serves no functional purpose, and is a visual aid only.
             */
            for (int i = 0; i < cellCount; i++) {
                Imgproc.rectangle(
                        input,
                        REGION_ARR_MOORING_POINTA[i],
                        REGION_ARR_MOORING_POINTB[i],
                        BLUE,
                        2);

//                Imgproc.putText(input, "" + (i+1), new Point(REGION_ARR_MOORING_POINTA[i].x+REGION_WIDTH/2.4, REGION_ARR_MOORING_POINTA[i].y+REGION_HEIGHT/2), Imgproc.FONT_ITALIC, 1.0, GREEN);
            }

            /*
             * Draw which box the item is in to the screen
             * Again, this serves no purpose beyond visual aid.
             */
            Imgproc.rectangle(
                    input,
                    REGION_ARR_MOORING_POINTA[savedLocation],
                    REGION_ARR_MOORING_POINTB[savedLocation],
                    GREEN,
                    -1);





            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }
    }


    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public int getCell()
    {
        return savedLocation;
    }

    /*
     * Return the mid point of the cells
     */
    public int getCellMidPoint(){return cellMidPoint;}

}