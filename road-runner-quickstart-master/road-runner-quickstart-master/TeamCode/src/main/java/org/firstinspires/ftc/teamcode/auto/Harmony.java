package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.HarpsichordStandardSymbiote;

import org.firstinspires.ftc.teamcode.symbiotes.vision.CombinedDetectorHandler;
import org.firstinspires.ftc.teamcode.util.MedJunctionData;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.enums.AllianceInfo;
import org.firstinspires.ftc.teamcode.util.enums.StartPositions;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "harmony")
public class Harmony extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    static final double FEET_PER_METER = 3.28084;

    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();
    //3 for high
    //2 for medium
    int autoMode = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        //Set up Drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Open Camera
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
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                Storage.setAlliance(AllianceInfo.BLUE);
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
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
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                Storage.setStartIndicator(StartPositions.AWAY);
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
        }

        clock1.reset();

        while(clock1.milliseconds() < 500);

        decision = false;

        //Set the auto mode
        while(!decision && !isStarted() && !isStopRequested()){
            telemetry.addData("Please select the autonomous mode: ", "");
            telemetry.addLine();
            telemetry.addData("A for High Junction, B for Medium.", "");
            telemetry.update();
            if(gamepad1.a || gamepad2.a){
                decision = true;
                autoMode = 3;
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                autoMode = 2;
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            }
        }

        //Set up the drive train
        drive.setPoseEstimate(Storage.getRealStartPos());

        //Set up the symbiote
        HarpsichordStandardSymbiote symbiote = new HarpsichordStandardSymbiote(drive);
        symbiote.setAutomaticMode(true);
        symbiote.setLiftTargetHeight(autoMode);



        //Initialize all trajectory holders
        Trajectory moveToJunctionMid = null;
        Trajectory goToJunction1 = null;
        Trajectory moveToMid = null;
        Trajectory goToParkArea2 = null;
        Trajectory goToParkArea3 = null;


        //********//
        switch(autoMode){
            case 2:
                moveToJunctionMid = drive.trajectoryBuilder(Storage.getRealStartPos())
                        .lineToLinearHeading(MedJunctionData.getData(1, 0))
                        .build();

                goToJunction1 = drive.trajectoryBuilder(moveToJunctionMid.end())
                        .lineToLinearHeading(MedJunctionData.getData(2, 0))
                        .build();

                moveToMid = drive.trajectoryBuilder(goToJunction1.end())
                        .lineToLinearHeading(MedJunctionData.getData(3, 0))
                        .build();

                goToParkArea2 = drive.trajectoryBuilder(moveToMid.end())
                        .lineToLinearHeading(MedJunctionData.getData(4, 0))
                        .build();

                goToParkArea3 = drive.trajectoryBuilder(moveToMid.end())
                        .lineToLinearHeading(MedJunctionData.getData(5, 0))
                        .build();
                break;
            case 3:
                break;
        }

        while(gamepad1.a != true && gamepad2.a != true){
            telemetry.addData("Please insert cone", "...");
            telemetry.addData("Press A to continue", ".");
            telemetry.update();
        }
        drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

        drive.outtakeClaw.setPosition(0);//Close the outtakeclaw


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

        /* Update the telemetry, and finalize the april tag snapshot */
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

        matchTimer.reset();
        telemetry.addData("Running...", "");
        telemetry.update();

        //Move to the junction
        drive.followTrajectory(moveToJunctionMid);
        drive.followTrajectory(goToJunction1);

        //Send the request to deliver
        symbiote.sendStatusRequest(1);
        symbiote.setLiftTargetHeight(2);
        //Wait until the symbiont has finished, updating the whole time
        clock1.reset();
        while(clock1.milliseconds() < 300 || (symbiote.isBusy() && opModeIsActive() && !isStopRequested())) {
            symbiote.updateAutonomousHarpsichordSymbiote(5, 2);
            drive.update();
            telemetry.addData("Cycling...", "First run");
            telemetry.update();
        }
        int stack = 5;
        //Cycle five times
        while(stack > 0){
            telemetry.addData("True Cycling...", stack);
            //Send the request to deliver
            symbiote.sendStatusRequest(3);
            //Wait until the symbiote has finished, updating the whole time
            clock1.reset();
            while(clock1.milliseconds() < 300 || (symbiote.isBusy() && opModeIsActive() && !isStopRequested())) {
                symbiote.updateAutonomousHarpsichordSymbiote(stack, 2);
            }
            stack--;
        }
        symbiote.sendStatusRequest(4);
        symbiote.updateAutonomousHarpsichordSymbiote(0, 0);
        drive.followTrajectory(moveToMid);
        //Move to the final parking location if it is not the center
        if(tagOfInterest.id == LEFT){
            drive.followTrajectory(goToParkArea3);
        }
        else if(tagOfInterest.id == RIGHT){
            drive.followTrajectory(goToParkArea2);
        }

        Storage.setCurrentPos(drive.getPoseEstimate());

        telemetry.addData("Match Time: ", matchTimer.milliseconds());
        telemetry.update();
        sleep(15000);


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


