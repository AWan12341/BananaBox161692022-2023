package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.HarpsichordStandardSymbiote;
import org.firstinspires.ftc.teamcode.symbiotes.vision.CombinedDetectorHandler;
import org.firstinspires.ftc.teamcode.symbiotes.vision.utility.TrackType;
import org.firstinspires.ftc.teamcode.util.Storage;

@TeleOp(group = "drive")
public class Orchestra extends LinearOpMode {

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
        HarpsichordStandardSymbiote symbiote = new HarpsichordStandardSymbiote(drive);

        double speedMod = .5;

        drive.blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        symbiote.setLiftTargetHeight(3);

        waitForStart();
        matchTimer.reset();


        while (!isStopRequested() && opModeIsActive()) {

            /** Update all symbiotes and drivers */
            drive.update();
//            Pose2d poseEstimate = drive.getPoseEstimate();
            symbiote.updateHarpsichordSymbiote();

//            if(matchTimer.seconds() > 89){
//                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
//            }



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
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedMod,
                                -gamepad1.left_stick_x * speedMod,
                                -gamepad1.right_stick_x * speedMod

                        )
                );



            //Automatic lift Controls
//            if(automaticDriveMode)
//            {
//                coneOffset = combinedDetectorHandler.getForwardConeOffset(1) * -.005;
//                if(testMode == 0){
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    0,
//                                    0,
//                                    -drive.getRawExternalHeading()
//
//                            )
//                    );
//                }
//                else if(testMode == 1){
//
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    0,
//                                    0,
//                                    coneOffset > -.1 && coneOffset < .1 ? 0 : coneOffset
//                            )
//                    );
//                }
//            }


            /** Operator Controls */
            //Automatic lift Controls

            if(gamepad2.a && gamepad2.b){
                symbiote.setLiftTargetHeight(1);
            }
            else if(gamepad2.a && gamepad2.y){
                symbiote.setLiftTargetHeight(2);
            }
            else if (gamepad2.a && gamepad2.x){
                symbiote.setLiftTargetHeight(3);
            }

                if(gamepad2.dpad_up && !symbiote.isBusy()) {
                    symbiote.sendStatusRequest(0);
                }
                else if (gamepad2.dpad_right && !symbiote.isBusy()) {
                    symbiote.sendStatusRequest(1);
                }
                else if (gamepad2.left_bumper && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(2);
                }
                else if ((gamepad2.right_bumper || gamepad2.dpad_down) && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(3);
                }

                else if(gamepad2.dpad_left && !symbiote.isBusy())
                    symbiote.sendStatusRequest(4);

//            if(gamepad2.x && gamepad1.x && symbiote.readyFlag){
//                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
//                symbiote = new OboeStandardSymbiote();
//                symbiote.setup(drive);
//            }
//            if(symbiote.readyFlag){
//                drive.blinkInRear.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//            }



            /** Operator Controls */
            //Automatic Lift Controls/Overrides
            if(gamepad2.a){
                symbiote.cyclingMode = true;
            }
            else if(gamepad2.b){
                symbiote.cyclingMode = false;
            }
            if(gamepad1.right_bumper){
                drive.intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drive.intakeSlide.setPower(0);
            }
            if(gamepad1.left_bumper){
                drive.intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.intakeSlide.setTargetPosition(0);
                drive.intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.intakeSlide.setPower(1);
            }



            /** Telemetry */
//            telemetry.addData("Cone Distance ", drive.forwardSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Intake Slide Current Pos ", drive.intakeSlide.getCurrentPosition());
            telemetry.addData("Current Prime State ", symbiote.primeState);
            telemetry.addData("Current Grab State ", symbiote.grabState);
            telemetry.addData("Current Delivery State ", symbiote.deliveryState);

            telemetry.update();

//            if(isStopRequested()) {
//                drive.lift.setTargetPosition(0);
//                sleep(1000);
//            }
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

