package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Disabled
public class MotorCalibration extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "liftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setPower(.5);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int val = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a && timer.milliseconds() > 200){
                val += 20;
                timer.reset();
            }
            else if(gamepad1.b && timer.milliseconds() > 200){
                val -= 20;
                timer.reset();
            }
            motor.setTargetPosition(val);

            telemetry.addData("Pos: ", motor.getCurrentPosition());
            telemetry.addData("Val: ", val);
            telemetry.update();
        }
    }
}
