package org.firstinspires.ftc.teamcode.symbiotes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HarpsichordStandardSymbiote {

    //Imports
    private SampleMecanumDrive drive;

    //Slide positions
    private int extendedPos = 370, retractedPos = 0, transferPos = 130, floatingTargetPos = 130;

    //Lift positions
    private int liftRestPosition = 300;
    private int liftHeight1Pos = 300, liftHeight2Pos = 600, liftHeight3Pos = 905;
    private int tempHeight = 905;

    //Outtake wrist
    private double outtakeWristOutPos = .2;
    private double outtakeWristInPos = .86;

    //Outtake Claw
    private double outtakeClawOpenPos = .75;
    private double outtakeClawClosedPos = 0;

    //Intake wrist
    private double intakeWristInPos = .15, intakeWristOutPos = .76, intakeWristMidPos = .5;

    //grippers
    private double leftGripperOpenPos = 1.0;
    private double leftGripperClosedPos = .75;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime rampDelay = new ElapsedTime();
    ElapsedTime requestDelay = new ElapsedTime();


    //Other
    boolean automaticMode = false;

    //Flags
    boolean shouldPrime = false, shouldGrab = false, shouldPrepareDelivery = false, shouldDeliver = false, shouldRest = false;
    boolean isBusy = false;
    public boolean readyFlag = false;

    //State Machine Enums
    public enum PRIMESTATE{
        RESET,
        EXTEND,
        CANCEL
    }

    public enum GRABSTATE{
        GRAB,
        RETRACT,
        PREPARE_TRANSFER,
        TRANSFER,
        LIFT,
        RESET,
        CANCEL
    }

    public enum DELIVERYSTATE{
        PREPARE,
        DELIVER,
        RESET,
        CANCEL
    }

    //State Machine Flags
    public PRIMESTATE primeState = PRIMESTATE.RESET;
    GRABSTATE grabState = GRABSTATE.GRAB;
    DELIVERYSTATE deliveryState = DELIVERYSTATE.PREPARE;



    public HarpsichordStandardSymbiote(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable

        setLiftTargetPosition(0, 1); //Return the lift to rest position
        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
        setIntakeExtension(0, 1); //Pull back the intake
        setIntakeWristPos(1);
        setIntakeGrippersPos(0);
        setIntakeWristUpperPos(1);
        rampDelay.reset();

        readyFlag = true;
    }


    public void updateViolaSymbiote(){

            if(shouldPrime){
                isBusy = true;
                //Should we reset?
                if(drive.intakeWrist.getPosition() < intakeWristOutPos && primeState == PRIMESTATE.RESET) {
                    setIntakeGrippersPos(2); //Close the gripper
                    setIntakeWristPos(0); //Lower the wrist
                    setLiftTargetPosition(0, 1); //Return lift to rest position
                }
                else {
                    primeState = PRIMESTATE.EXTEND;
                }

                //Should we extend under automatic mode?
                if (automaticMode && drive.intakeSlide.getCurrentPosition() < floatingTargetPos && primeState == PRIMESTATE.EXTEND){
                    setIntakeExtension(3, 1); //Extend the intake, using the distance sensor
                    setIntakeGrippersPos(0); //Open the grippers
                }
                //Should we extend under manual mode?
                else if (!automaticMode && drive.intakeSlide.getCurrentPosition() < extendedPos && primeState == PRIMESTATE.EXTEND){
                    setIntakeExtension(2, 1); //Extend the intake
                    setIntakeGrippersPos(0); //Open the grippers
                }
                //Now that we're finished, set everything back up for next time.
                else{
                    primeState = PRIMESTATE.RESET;
                    shouldPrime = false;
                    isBusy = false;
                    if(automaticMode){
                        shouldGrab = true;
                    }
                }
            }

            else if(shouldGrab){
                //Should we close the gripper?
                if(grabState == GRABSTATE.GRAB && drive.intakeGrip2.getPosition() < leftGripperClosedPos){
                    setIntakeGrippersPos(2); //close the gripper
                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos); //Ensure the outtake is properly set up
                }
                else {
                    grabState = GRABSTATE.RETRACT;
                }
                //Should we retract the intake?
                if(grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() > transferPos){
                    setIntakeWristPos(1);//Send wrist to a mid point
                    setIntakeExtension(0, .5); //Pull back the intake
                }
                else{
                    grabState = GRABSTATE.PREPARE_TRANSFER;
                }
                //Should we prepare to transfer?
                if(grabState == GRABSTATE.PREPARE_TRANSFER && drive.intakeWrist.getPosition() > intakeWristInPos){
                    setIntakeWristPos(2); //Pull the wrist up
                }
                else {
                    grabState = GRABSTATE.TRANSFER;
                }
                //Should we transfer?
                if(grabState == GRABSTATE.TRANSFER && drive.intakeGrip2.getPosition() > leftGripperOpenPos){
                    setIntakeGrippersPos(3); //Open the grippers
                }
                else {
                    grabState = GRABSTATE.LIFT;
                }
                //Should we lift up?
                if(grabState == GRABSTATE.RETRACT && drive.lift.getCurrentPosition() < tempHeight){
                    setIntakeWristPos(1);//Send wrist to a mid point
                    setIntakeExtension(0, .5); //Pull back the intake
                }
                else{
                    grabState = GRABSTATE.PREPARE_TRANSFER;
                }
            }
            else if (shouldPrepareDelivery){

            }
            else if (shouldDeliver){

            }
            else if(shouldRest){
                setLiftTargetPosition(0, 1); //Return the lift to rest position
                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                setIntakeExtension(0, .5); //Pull back the intake
                setIntakeWristPos(1);
                setIntakeGrippersPos(0);
                shouldRest = false;
            }

    }

    public void updateAutonomousViolaSymbiote(int stage, int height){

        if(stage == 1){

//            setIntakeExtension(1); //Extend the intake
            setIntakeWristPos(height+2);//Lower the wrist
            setIntakeWristUpperPos(height); //Angle the upper wrist
            setIntakeGrippersPos(0); //Open the grippers

            wait(800);

            setIntakeGrippersPos(2); //close the gripper

            wait(400);

            setIntakeWristPos(1);//Mid point
            setIntakeWristUpperPos(0); // Return the upper wrist to default

            wait(200);

            if(height == 5) {
//                setIntakeExtension(2); //Pull back the intake
            }
            else {
//                drive.intakeExtension.setPosition(.22);
//                drive.intakeExtensionRight.setPosition(.82);
            }

            wait(100);

            setIntakeWristPos(2); //Pull the wrist up

            wait(600);

            setIntakeGrippersPos(0); //Open the grippers

            wait(600);

            setLiftTargetPosition(3, 1); //Lift up the lift

            wait(1600);

//            while(drive.lift.getTargetPosition() < (4600 - liftRestPosition)){drive.update();}

            setIntakeGrippersPos(2); //close the gripper
            setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
            setIntakeWristPos(1); //Wrist mid point

            wait(700);


        }
        else if (stage == 2 || stage == 3){

            if(stage == 3) {
                setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                setLiftTargetPosition(3, 1); //Lift up the lift

//                while(drive.lift.getTargetPosition() <= (4600 - liftRestPosition)){drive.update();}

                wait(2000);
            }

            setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);

            wait(800);

            setLiftTargetPosition(0, 1); //Return the lift to rest position
            setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);

        }
        else if(stage == 4){
            setLiftTargetPosition(0, 1); //Return the lift to rest position
            setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
//            setIntakeExtension(0); //Pull back the intake
            setIntakeWristPos(1);
            setIntakeGrippersPos(0);
            shouldRest = false;
        }

    }

    public boolean isBusy(){
        return isBusy;
    }

    public void wait(int time){
        clock1.reset();
        while(clock1.milliseconds() < time){}
    }

    public void setLiftTargetHeight(int targ){
        switch(targ){
            case 1:
                tempHeight = liftHeight1Pos;
                break;
            case 2:
                tempHeight = liftHeight2Pos;
                break;
            case 3:
                tempHeight = liftHeight3Pos;
                break;
        }
    }

    public void sendStatusRequest(int status){
        if(!drive.isBusy()) {
            switch (status) {
                case 0:
                    shouldPrime = true;
                    break;
                case 1:
                    shouldGrab = true;
                    break;
                case 2:
                    shouldPrepareDelivery = true;
                    break;
                case 3:
                    shouldDeliver = true;
                    break;
                case 4:
                    shouldRest = true;
                    break;
            }
        }
    }

    public void setAutomaticMode(boolean mode){
        automaticMode = mode;
    }

    //1 = Level 1
    //2 = Level 2
    //3 = Level 3
    //4 = Level 4
    public void setIntakeWristUpperPos(int position){
        switch(position){
            case 1:
                drive.intakeWristUpper.setPosition(.48);
                break;
            case 2:
                drive.intakeWristUpper.setPosition(.56);
                break;
            case 3:
                drive.intakeWristUpper.setPosition(.58);
                break;
            case 4:
                drive.intakeWristUpper.setPosition(.6);
                break;
            case 5:
                drive.intakeWristUpper.setPosition(.6);
                break;
        }
    }

    //0 = open
    //1 = mid
    //2 = closed

    //INTAKE GRIP 1 IS RIGHT
    public void setIntakeGrippersPos(int position){
        switch(position){
            case 0:
                drive.intakeGrip1.setPosition(.05);
                drive.intakeGrip2.setPosition(leftGripperOpenPos);
                break;
            case 1:
                //Todo: Find a mid position. Low importance
                drive.intakeGrip1.setPosition(.2);
                drive.intakeGrip2.setPosition(.8);
                break;
            case 2:
                drive.intakeGrip1.setPosition(.33);
                drive.intakeGrip2.setPosition(leftGripperClosedPos);
                break;
        }
    }

    //0 = out
    //1 = mid
    //2 = in
    //3 = Level 1
    //4 = Level 2
    //5 = Level 3
    //6 = Level 4
    //7 = Level 5
    public void setIntakeWristPos(int position){
        switch(position){
            case 0:
                drive.intakeWrist.setPosition(intakeWristOutPos);
                break;
            case 1:
                drive.intakeWrist.setPosition(intakeWristMidPos);
                break;
            case 2:
                drive.intakeWrist.setPosition(intakeWristInPos);
                break;
            case 3:
                drive.intakeWrist.setPosition(.78);
                break;
            case 4:
                drive.intakeWrist.setPosition(.7);
                break;
            case 5:
                drive.intakeWrist.setPosition(.67);
                break;
            case 6:
                drive.intakeWrist.setPosition(.6);
                break;
            case 7:
                drive.intakeWrist.setPosition(.56);
                break;
        }
    }

    //Left as two separate values because we do weird things with it
    public void setOuttakePos(double pos, double pos2){
        drive.outtakeWrist.setPosition(pos);
        drive.outtakeClaw.setPosition(pos2);
    }

    //0 = fully retracted
    //1 = transfer pose direct
    //2 = fully extended
    //3 = fully extended position with damping
    public void setIntakeExtension(int position, double pow){



        switch (position){
            case 0:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(retractedPos);
                break;
            case 1:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(transferPos);
                break;
            case 2:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(extendedPos);
                break;
            case 3:
                drive.intakeSlide.setPower(pow);
//                if(drive.intakeSlide.getCurrentPosition() < 170 && drive.forwardSensor.getDistance(DistanceUnit.INCH) > 2.5){
//                    drive.intakeSlide.setPower(pow * (drive.forwardSensor.getDistance(DistanceUnit.INCH) * (drive.forwardSensor.getDistance(DistanceUnit.INCH) > 10?.05:.05)));
//                }
//                else {
//                    drive.intakeSlide.setPower(0);
//                }
                floatingTargetPos = (int)(drive.forwardSensor.getDistance(DistanceUnit.INCH) * 8.5);
                drive.intakeSlide.setTargetPosition(floatingTargetPos);
                break;
        }
    }

    //0 = Reset
    //1 = Junction height 1
    //2 = Junction height 2
    //3 = Junction height 3;
    public void setLiftTargetPosition(int level, double pow){

        drive.lift.setPower(pow);

        switch(level){
            case 0:
                drive.lift.setTargetPosition(liftRestPosition);
                break;
            case 1:
                drive.lift.setTargetPosition(liftHeight1Pos);
                break;
            case 2:
                drive.lift.setTargetPosition(liftHeight2Pos);
                break;
            case 3:
                drive.lift.setTargetPosition(liftHeight3Pos);
                break;
        }
    }


}
