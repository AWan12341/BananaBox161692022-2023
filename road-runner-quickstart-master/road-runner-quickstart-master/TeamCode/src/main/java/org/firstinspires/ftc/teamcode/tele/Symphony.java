package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.OboeStandardSymbiote;
import org.firstinspires.ftc.teamcode.symbiotes.ViolaStandardSymbiote;
import org.firstinspires.ftc.teamcode.symbiotes.vision.CombinedDetectorHandler;
import org.firstinspires.ftc.teamcode.symbiotes.vision.utility.TrackType;
import org.firstinspires.ftc.teamcode.util.Storage;

@TeleOp(group = "drive")
public class Symphony extends LinearOpMode {

    int testMode = 0;

    boolean automaticMode = false, automaticDriveMode = false;
    boolean switchCase = false;
    double coneOffset;
    double poleOffset;
    double errorToZeroHeading;
    boolean switchSys = false;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime matchTimer = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        //Set up the drive train
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Storage.getCurrentPos());

        CombinedDetectorHandler combinedDetectorHandler = new CombinedDetectorHandler(hardwareMap, "webcam 1", "webcam 2","webcam 3");
        combinedDetectorHandler.init();
        combinedDetectorHandler.setTrackingTypes(TrackType.CONE, TrackType.POLE, TrackType.POLE);

        //Set up the symbiote
        OboeStandardSymbiote symbiote = new OboeStandardSymbiote();

        symbiote.setup(drive);

        double speedMod = .5;

        drive.blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        symbiote.setLiftTargetHeight(3);

        waitForStart();
        matchTimer.reset();


        while (!isStopRequested() && opModeIsActive()) {

            /** Update all symbiotes and drivers */
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            symbiote.updateViolaSymbiote();

            if(matchTimer.seconds() > 89){
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }



            /** Driver Controls */
            if(gamepad1.x)
                speedMod = 1;
            else if(gamepad1.a)
                speedMod = .75;
            else if (gamepad1.b)
                speedMod = .5;
            else if (gamepad1.y)
                speedMod = .25;

            //Manual
            if(!automaticDriveMode)
            {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * speedMod,
                                gamepad1.left_stick_x * speedMod,
                                gamepad1.right_stick_x * speedMod

                        )
                );
            }


            //Automatic lift Controls
            if(automaticDriveMode)
            {
                coneOffset = combinedDetectorHandler.getForwardConeOffset(1) * -.005;
                poleOffset = combinedDetectorHandler.getPoleParallax() * -.005;
                if(testMode == 0){
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0,
                                    -drive.getRawExternalHeading()

                            )
                    );
                }
                else if(testMode == 1){

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    0,
                                    0,
                                    coneOffset > -.1 && coneOffset < .1 ? 0 : coneOffset

                            )
                    );
                }
            }


            /** Operator Controls */
            //Automatic lift Controls
            if(automaticMode)
            {
                symbiote.sendStatusRequest(0);
            }

            if(gamepad2.a && gamepad2.b){
                symbiote.setLiftTargetHeight(1);
            }
            else if(gamepad2.a && gamepad2.y){
                symbiote.setLiftTargetHeight(2);
            }
            else if (gamepad2.a && gamepad2.x){
                symbiote.setLiftTargetHeight(3);
            }

            //Manual Lift Controls
            if(!automaticMode){
                if(gamepad2.dpad_up && !symbiote.isBusy())
                    symbiote.sendStatusRequest(0);

                else if (gamepad2.dpad_right && !symbiote.isBusy())
                    symbiote.sendStatusRequest(1);

                else if (gamepad2.left_bumper && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(2);
                }
                else if (gamepad2.right_bumper && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(3);
                }

//                else if (gamepad2.dpad_down && switchSys && !symbiote.isBusy() && clock1.milliseconds() > 500){
//
//                    switchSys = false;
//                    clock1.reset();
//                }
//
//                else if (gamepad2.dpad_down && !switchSys && clock1.milliseconds() > 500 && !symbiote.isBusy()){
//
//                    switchSys = true;
//                    clock1.reset();
//                }

                else if(gamepad2.dpad_left && !symbiote.isBusy())
                    symbiote.sendStatusRequest(4);
            }
            if(gamepad2.x && gamepad1.x && symbiote.readyFlag){
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
                symbiote = new OboeStandardSymbiote();
                symbiote.setup(drive);
            }
            if(symbiote.readyFlag){
                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }



            /** Operator Controls */
            //Automatic Lift Controls/Overrides
//            if(gamepad2.a){
//                symbiote.setAutomaticMode(true);
//                automaticMode = true;
//            }
//            else if(gamepad2.b){
//                symbiote.setAutomaticMode(false);
//                automaticMode = false;
//            }



            /** Telemetry */
            telemetry.addData("Pole Parallax ", poleOffset);
            telemetry.addData("Pow ", coneOffset);
            telemetry.addData("front Cone ", combinedDetectorHandler.getForwardConeOffset(1));
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.update();
        }
    }

    public boolean getStateChange(){
        if(gamepad2.dpad_down != switchCase){
            return true;
        }
        switchCase = gamepad2.dpad_down;
        return false;
    }

}



//********* Boneyard **********

