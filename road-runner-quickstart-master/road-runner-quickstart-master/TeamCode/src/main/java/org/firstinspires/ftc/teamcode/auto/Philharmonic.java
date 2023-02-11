package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.vision.deprecated.ViolaStandardSymbiote;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.enums.AllianceInfo;
import org.firstinspires.ftc.teamcode.util.enums.StartPositions;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "harmony")
@Disabled
public class Philharmonic extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 622.001;
    double fy = 622.001;
    double cx = 319.803;
    double cy = 241.251;

    // UNITS ARE METERS
    double tagsize = 0.04445;

//    int ID_TAG_OF_INTEREST = 0; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    int tagOfInterestID = 0;

    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    ElapsedTime clock1 = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Set our alliance color

        boolean decision = false;
        while(!decision && !isStarted() && !isStopRequested()){
            telemetry.addData("Please select Alliance: ", "");
            telemetry.addLine();
            telemetry.addData("A for Red, B for Blue.", "");
            telemetry.update();
            if(gamepad1.a || gamepad2.a){
                decision = true;
                Storage.setAlliance(AllianceInfo.RED);
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                Storage.setAlliance(AllianceInfo.BLUE);
            }
        }
        clock1.reset();

        while(clock1.milliseconds() < 500);

        decision = false;

        //Set the start position
        while(!decision && !isStarted() && !isStopRequested()){
            telemetry.addData("Please select Start Position: ", "");
            telemetry.addLine();
            telemetry.addData("A for Audience, B for Away.", "");
            telemetry.update();
            if(gamepad1.a || gamepad2.a){
                decision = true;
                Storage.setStartIndicator(StartPositions.AUDIENCE);
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                Storage.setStartIndicator(StartPositions.AWAY);
            }
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        //Set up the drive train
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Storage.getRealStartPos());

        //Set up the symbiote
        ViolaStandardSymbiote symbiote = new ViolaStandardSymbiote();
        symbiote.setup(drive);
        symbiote.setAutomaticMode(true);

        //Generate trajectories for Blue Audience
//        Trajectory traj1BlueAudience = drive.trajectoryBuilder(Storage.getRealStartPos())
//                .lineToLinearHeading(new Pose2d(35, 0, Math.toRadians(10)))
//                .build();
//
//        Trajectory traj1BlueAudience1 = drive.trajectoryBuilder(traj1BlueAudience.end())
//                .lineToLinearHeading(new Pose2d(35, 9, Math.toRadians(10)))
//                .build();
//
//        Trajectory traj1BlueAudience2 = drive.trajectoryBuilder(traj1BlueAudience1.end())
//                .lineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(-190)))
//                .build();


        //Generate trajectories for Red Audience
        Trajectory traj1RedAudiencestart = drive.trajectoryBuilder(Storage.getRealStartPos())
                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(90)))
                .build();

        //Deliver preload
        Trajectory traj1RedAudience = drive.trajectoryBuilder(traj1RedAudiencestart.end())
                .lineToLinearHeading(new Pose2d(33, 0, Math.toRadians(-10)))
                .build();

//        Trajectory traj1RedAudience1 = drive.trajectoryBuilder(traj1RedAudience.end())
//                .lineToLinearHeading((new Pose2d(37, -15, Math.toRadians(0))))
//                .build();
//
//        Trajectory traj1RedAudience2 = drive.trajectoryBuilder(traj1RedAudience1.end())
//                .lineToLinearHeading((new Pose2d(42, -15, Math.toRadians(0))))
//                .build();
//
//        //Grab cone
//        Trajectory traj1RedAudience3 = drive.trajectoryBuilder(traj1RedAudience2.end())
//                .lineToLinearHeading((new Pose2d(34, -15, Math.toRadians(0))))
//                .build();
//
//        Trajectory traj1RedAudience4 = drive.trajectoryBuilder(traj1RedAudience3.end())
//                .lineToLinearHeading((new Pose2d(30, -6, Math.toRadians(-10))))
//                .build();



        Trajectory traj2RedAudience1 = drive.trajectoryBuilder(traj1RedAudience.end())
                .lineToLinearHeading((new Pose2d(35, -35, Math.toRadians(90))))
                .build();

        Trajectory traj2RedAudienceRight = drive.trajectoryBuilder(traj2RedAudience1.end())
                .lineToLinearHeading((new Pose2d(60, -35, Math.toRadians(90))))
                .build();

        Trajectory traj2RedAudienceLeft = drive.trajectoryBuilder(traj2RedAudience1.end())
                .lineToLinearHeading((new Pose2d(10, -35, Math.toRadians(90))))
                .build();

        //Second cone

//        Trajectory traj2RedAudience1 = drive.trajectoryBuilder(traj1RedAudience4.end())
//                .lineToLinearHeading((new Pose2d(34, -12, Math.toRadians(0))))
//                .build();
//
//        Trajectory traj2RedAudience2 = drive.trajectoryBuilder(traj2RedAudience1.end())
//                .lineToLinearHeading((new Pose2d(38, -12, Math.toRadians(0))))
//                .build();
//
//        Trajectory traj2RedAudience3 = drive.trajectoryBuilder(traj2RedAudience2.end())
//                .lineToLinearHeading((new Pose2d(34, -12, Math.toRadians(0))))
//                .build();
//
//        Trajectory traj2RedAudience4 = drive.trajectoryBuilder(traj2RedAudience3.end())
//                .lineToLinearHeading((new Pose2d(30, -2, Math.toRadians(-10))))
//                .build();


//        waitForStart();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested() && !opModeIsActive())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(Storage.getAlliance() == AllianceInfo.BLUE && Storage.getStartIndicator() == StartPositions.AUDIENCE) {

//            drive.followTrajectory(traj1BlueAudience);
        }
        else if(Storage.getAlliance() == AllianceInfo.RED && Storage.getStartIndicator() == StartPositions.AUDIENCE) {

            drive.followTrajectory(traj1RedAudiencestart);

            drive.followTrajectory(traj1RedAudience);

            symbiote.updateAutonomousViolaSymbiote(3, 1); //Deliver the preload

//            drive.followTrajectory(traj1RedAudience1);
//            drive.followTrajectory(traj1RedAudience2);
//
//            symbiote.updateAutonomousViolaSymbiote(1, 5);
//            drive.followTrajectory(traj1RedAudience3);
//            drive.followTrajectory(traj1RedAudience4);
//            symbiote.updateAutonomousViolaSymbiote(2, 5);

            drive.followTrajectory(traj2RedAudience1);

            if(tagOfInterest.id == LEFT){
                drive.followTrajectory(traj2RedAudienceLeft);
            }
            else if (tagOfInterest.id == MIDDLE){
                //Do nothing!
            }
            else if (tagOfInterest.id == RIGHT){
                drive.followTrajectory(traj2RedAudienceRight);
            }

        }





        //Grab a second cone

//        drive.followTrajectory(traj2RedAudience1);
//        drive.followTrajectory(traj2RedAudience2);
//
//        symbiote.updateAutonomousViolaSymbiote(1, 4);
//        drive.followTrajectory(traj2RedAudience3);
//        drive.followTrajectory(traj2RedAudience4);
//        symbiote.updateAutonomousViolaSymbiote(2, 4);


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


