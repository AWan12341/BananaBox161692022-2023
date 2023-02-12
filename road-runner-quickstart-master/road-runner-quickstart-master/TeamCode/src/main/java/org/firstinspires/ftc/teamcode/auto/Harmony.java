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

    ElapsedTime clock1 = new ElapsedTime();
    int aprilTagOuput;
    int autoMode = 0;
    //3 for high
    //2 for medium


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CombinedDetectorHandler vision = new CombinedDetectorHandler(hardwareMap, "webcam 1", "webcam 2", "webcam 3");

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
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                Storage.setStartIndicator(StartPositions.AWAY);
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
            }
            else if (gamepad1.b || gamepad2.b){
                decision = true;
                autoMode = 2;
            }
        }

        //Set up the drive train
        drive.setPoseEstimate(Storage.getRealStartPos());

        //Set up the symbiote
        HarpsichordStandardSymbiote symbiote = new HarpsichordStandardSymbiote(drive);
        symbiote.setAutomaticMode(true);
        symbiote.setLiftTargetHeight(autoMode);

        //Initialize all trajectory holders
        Trajectory goToJunction1 = null;
        Trajectory moveAcrossField = null;
        Trajectory goToJunction2 = null;

        //********//
        switch(autoMode){
            case 2:
                goToJunction1 = drive.trajectoryBuilder(Storage.getRealStartPos())
                        .lineToLinearHeading(MedJunctionData.getData(1))
                        .build();
                moveAcrossField = drive.trajectoryBuilder(goToJunction1.end())
                        .lineToLinearHeading(MedJunctionData.getData(2))
                        .build();
                goToJunction2 = drive.trajectoryBuilder(moveAcrossField.end())
                        .lineToLinearHeading(MedJunctionData.getData(3))
                        .build();
                break;
            case 3:
                break;
        }


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested() && !opModeIsActive())
        {
            telemetry.addData("Waiting for Start", "");
            aprilTagOuput = vision.getAprilTagPipeOutput();
            if(aprilTagOuput == 6){
                telemetry.addData("No Tag Found", aprilTagOuput);
            }
            else {
                telemetry.addData("Tag Found! Tag ID: ", aprilTagOuput);
            }
            telemetry.update();
        }
        telemetry.addData("Running...", "");
        telemetry.update();

        symbiote.sendStatusRequest(0);

        clock1.reset();
        while(clock1.milliseconds() < 300 || (symbiote.isBusy() && opModeIsActive() && !isStopRequested())) {
            symbiote.updateAutonomousHarpsichordSymbiote(0, 2);
            drive.update();
            telemetry.addData("Prepping...", "First run");
            telemetry.update();
        }
        //Move to the junction
        drive.followTrajectory(goToJunction1);
        //Send the request to deliver
        symbiote.sendStatusRequest(1);
        symbiote.setLiftTargetHeight(2);
        //Wait until the symbiont has finished, updating the whole time
        clock1.reset();
        while(clock1.milliseconds() < 300 || (symbiote.isBusy() && opModeIsActive() && !isStopRequested())) {
            symbiote.updateAutonomousHarpsichordSymbiote(0, 2);
            drive.update();
            telemetry.addData("Cycling...", "First run");
            telemetry.update();
        }
        int stack = 5;
        //Cycle five times
        while(stack > 0){
            telemetry.addData("True Cycling...", "");
            //Send the request to deliver
            symbiote.sendStatusRequest(3);
            //Wait until the symbiote has finished, updating the whole time
            clock1.reset();
            while(clock1.milliseconds() < 300 || (symbiote.isBusy() && opModeIsActive() && !isStopRequested())) {
                symbiote.updateAutonomousHarpsichordSymbiote(stack, 2);
            }
            stack--;
        }


    }

}


