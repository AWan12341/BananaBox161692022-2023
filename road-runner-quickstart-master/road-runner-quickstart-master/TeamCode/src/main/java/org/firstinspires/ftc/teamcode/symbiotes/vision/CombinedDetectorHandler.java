package org.firstinspires.ftc.teamcode.symbiotes.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.symbiotes.vision.utility.TrackType;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class CombinedDetectorHandler {

    OpenCvCamera webcamFront;
    OpenCvCamera webcamRearLeft;
    OpenCvCamera webcamRearRight;

    private final String webcamFrontName, webcamRearLeftName, webcamRearRightName;
    private final HardwareMap hardwareMap;
    private CombinedTracker WFPipe = new CombinedTracker(), WRLPipe = new CombinedTracker(), WRRPipe = new CombinedTracker();
    //The X resolution of all cameras
    private int xRes = 320;

    //The compensated centers of each parallax camera
    private int leftCenterOffset = -260, rightCenterOffset = -90, forwardConeOffset = 88;

    CombinedTracker.DETECT_COLOR color = CombinedTracker.DETECT_COLOR.BLUE;

//    private double leftCamDistFromParallel = 5, rightCamDistFromParallel = -5;

    static final double FEET_PER_METER = 3.28084;



//    int ID_TAG_OF_INTEREST = 0; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    int tagOfInterestID = 0;

    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    public CombinedDetectorHandler(HardwareMap hardwareMap, String webcamFrontName, String webcamRearLeftName, String webcamRearRightName) {
        this.hardwareMap = hardwareMap;
        this.webcamFrontName = webcamFrontName;
        this.webcamRearLeftName = webcamRearLeftName;
        this.webcamRearRightName = webcamRearRightName;
    }

    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);

        webcamFront.setPipeline(WFPipe);
        webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcamFront.startStreaming(xRes, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        /**
//         * This is the only thing you need to do differently when using multiple cameras.
//         * Instead of obtaining the camera monitor view and directly passing that to the
//         * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
//         * on that view in order to split that view into multiple equal-sized child views,
//         * and then pass those child views to the constructor.
//         */
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
//                .splitLayoutForMultipleViewports(
//                        cameraMonitorViewId, //The container we're splitting
//                        1, //The number of sub-containers to create
//                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally
//        webcamFront = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamFrontName), viewportContainerIds[0]);
////        webcamRearLeft = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamRearLeftName), viewportContainerIds[1]);
////        webcamRearRight = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamRearRightName), viewportContainerIds[2]);
//
//        webcamFront.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcamFront.setPipeline(WFPipe);
//                webcamFront.startStreaming(xRes, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//        webcamRearLeft.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcamRearLeft.setPipeline(WRLPipe);
//                webcamRearLeft.startStreaming(xRes, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//        webcamRearRight.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcamRearRight.setPipeline(WRRPipe);
//                webcamRearRight.startStreaming(xRes, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
    }

    //Set the type of objects we're looking for.
    //Options are: Cone, Pole, Signal, None
    public void setTrackingTypes(TrackType front, TrackType RearLeft, TrackType RearRight){
        WFPipe.setTrackType(front);
        WRLPipe.setTrackType(RearLeft);
        WRRPipe.setTrackType(RearRight);
    }

    //TODO: Optimize this
    public int getForwardConeOffset(int colorIn){
        switch(colorIn){
            case 1:
                color = CombinedTracker.DETECT_COLOR.BLUE;
                break;
            case 2:
                color = CombinedTracker.DETECT_COLOR.RED;
                break;
            case 3:
                color = CombinedTracker.DETECT_COLOR.BOTH;
                break;
        }
        WFPipe.setTrackType(TrackType.CONE);
        WFPipe.setDetect_Color(color);
        int center = WFPipe.getTrackingCenter() - forwardConeOffset;
        return center;
    }

    public int getPoleParallax(){
        WRLPipe.setTrackType(TrackType.POLE);
        WRRPipe.setTrackType(TrackType.POLE);

        int LeftCamCenter = WRLPipe.getTrackingCenter() - 50;
        int RightCamCenter = WRRPipe.getTrackingCenter();

        //Calculate Error
        return (LeftCamCenter - RightCamCenter)/2;
    }

    public int getAprilTagPipeOutput() {
//        WFPipe.setTrackType(TrackType.SLEEVE);

        ArrayList<AprilTagDetection> currentDetections = WFPipe.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;

                    break;
                }
            }

            if(tagFound)
            {
                return tagOfInterest.id;
            }
            else
            {

                if(tagOfInterest == null)
                {
                    return 6;
                }
                else
                {
                    return tagOfInterest.id;
                }
            }

        }
        else
        {
            if(tagOfInterest == null)
            {
                return 6;
            }
            else
            {
                return tagOfInterest.id;
            }

        }

    }


}
