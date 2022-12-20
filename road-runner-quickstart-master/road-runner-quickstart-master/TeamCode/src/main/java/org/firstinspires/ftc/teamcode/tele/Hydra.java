package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.Remora;

@TeleOp(group = "drive")
public class Hydra extends LinearOpMode {

    boolean switchSys = false;

    ElapsedTime clock1 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Remora symbiote = new Remora();

        symbiote.setup(drive);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speedMod = .5;

        waitForStart();

        while (!isStopRequested()) {
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

            drive.update();

            if(gamepad2.dpad_up){
                symbiote.primeIntake(speedMod, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            if (gamepad2.dpad_right)
                symbiote.grabAndPrep(speedMod, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if (gamepad2.dpad_down && switchSys == true && clock1.milliseconds() > 300){
                symbiote.deliver(speedMod, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                switchSys = false;
                clock1.reset();
            }
            else if (gamepad2.dpad_down && switchSys == false && clock1.milliseconds() > 300){
                symbiote.lesserDelivery();
                switchSys = true;
                clock1.reset();
            }
            if(gamepad2.dpad_left)
                symbiote.restMode();


            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
