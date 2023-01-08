package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoCalibrator extends LinearOpMode {
    public Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        double servoPos;
        servo = hardwareMap.get(Servo.class, "intakeGrip2");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            servo.setPosition(gamepad1.left_stick_y);
            telemetry.addData("Pos:", servo.getPosition());
            telemetry.update();
        }
    }
}
