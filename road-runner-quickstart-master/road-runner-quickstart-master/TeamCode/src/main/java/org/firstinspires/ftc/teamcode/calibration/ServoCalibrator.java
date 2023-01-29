package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoCalibrator extends LinearOpMode {
    public Servo servo, servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        float val = 0;
        float val2 = 0;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        timer.reset();
        timer2.reset();

        servo = hardwareMap.get(Servo.class, "outtakeWrist");
        servo2 = hardwareMap.get(Servo.class, "outtakeClaw");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            if(gamepad1.a && timer.milliseconds() > 200){
                val += .02;
                timer.reset();
            }
            else if(gamepad1.b && timer.milliseconds() > 200){
                val -= .02;
                timer.reset();
            }

            if(gamepad1.x && timer2.milliseconds() > 200){
                val2 += .02;
                timer2.reset();
            }
            else if(gamepad1.y && timer2.milliseconds() > 200){
                val2 -= .02;
                timer2.reset();
            }
            servo.setPosition(val);
            servo2.setPosition(val2);
            telemetry.addData("TargPos:", val);
            telemetry.addData("TargPos2:", val2);
            telemetry.addData("Pos:", servo.getPosition());
            telemetry.addData("Pos2:", servo2.getPosition());
            telemetry.update();
        }
    }
}
