package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.vision.deprecated.ViolaStandardSymbiote;

@TeleOp(group = "drive")
@Disabled
public class Hydra extends LinearOpMode {

    boolean switchSys = false;

    boolean automaticMode = false;

    double targetHeading;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        //Set up the drive train
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set up the symbiote
        ViolaStandardSymbiote symbiote = new ViolaStandardSymbiote();

        symbiote.setup(drive);

        double speedMod = .5;

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            /** Update all symbiotes and drivers */
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            symbiote.updateViolaSymbiote();


            /** Driver Controls */
            if(gamepad1.x)
                speedMod = 1;
            else if(gamepad1.a)
                speedMod = .75;
            else if (gamepad1.b)
                speedMod = .5;
            else if (gamepad1.y)
                speedMod = .25;

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speedMod,
                                -gamepad1.left_stick_x * speedMod,
                                -gamepad1.right_stick_x * speedMod

                        )
                );



            //Automatic lift Controls
            if(automaticMode)
            {
                symbiote.sendStatusRequest(0);
            }

            //Manual Lift Controls
            if(!automaticMode){
                if(gamepad2.dpad_up && !symbiote.isBusy())
                    symbiote.sendStatusRequest(0);

                else if (gamepad2.dpad_right && !symbiote.isBusy())
                    symbiote.sendStatusRequest(1);

                else if (gamepad2.dpad_down && switchSys && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(2);
                    switchSys = false;
                    clock1.reset();
                }

                else if (gamepad2.dpad_down && !switchSys && clock1.milliseconds() > 300 && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(3);
                    switchSys = true;
//                    clock1.reset();
                }

                else if(gamepad2.dpad_left && !symbiote.isBusy())
                    symbiote.sendStatusRequest(4);
            }



            /** Operator Controls */
            //Automatic Lift Controls/Overrides
            if(gamepad2.a){
                symbiote.setAutomaticMode(true);
                automaticMode = true;
            }
            else if(gamepad2.b){
                symbiote.setAutomaticMode(false);
                automaticMode = false;
            }



            /** Telemetry */
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.update();
        }
    }
}

//********* Boneyard **********
//            else if (driveAuto){
//                errorToPole = vision.getPoleParallax()-90;
//                heading = poseEstimate.getHeading();
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                0,
//                                errorToPole > 100 ? errorToPole * -.002 : errorToPole * -.005,
//                                (heading > targetHeading+5 || heading < targetHeading-5) ? (heading * -1) * .002 : 0
//                        )
//                );
//            }


//            if(gamepad1.dpad_down){
//                driveAuto = true;
//            }
//            else if (gamepad1.dpad_up){
//                driveAuto = false;
//            }
