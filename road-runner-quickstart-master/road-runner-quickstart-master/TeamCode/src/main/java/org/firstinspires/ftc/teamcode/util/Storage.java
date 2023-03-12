package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.enums.AllianceInfo;
import org.firstinspires.ftc.teamcode.util.enums.StartPositions;

public class Storage {

    static AllianceInfo alliance = AllianceInfo.NONE;

    static StartPositions start = StartPositions.NONE;

    static Pose2d startPosAudienceBlue = new Pose2d(-35, 60, Math.toRadians(-90)); //READY

    static Pose2d startPosAudienceRed = new Pose2d(-35, -60, Math.toRadians(90)); //READY

    static Pose2d startPosAwayBlue = new Pose2d(35, 60, Math.toRadians(-90)); //READY

    static Pose2d startPosAwayRed = new Pose2d(35, -60, Math.toRadians(90)); //READY

    static Pose2d currentPos = new Pose2d(0, 0, Math.toRadians(0));

    public static Pose2d getCurrentPos(){
        return currentPos;
    }

    public static void setCurrentPos(Pose2d temp){
        currentPos = temp;
    }


    public static Pose2d getRealStartPos(){
        if(alliance == AllianceInfo.BLUE) {
            if (start == StartPositions.AUDIENCE)
                return startPosAudienceBlue;
            else if (start == StartPositions.AWAY)
                return startPosAwayBlue;
        }
        else if(alliance == AllianceInfo.RED){
            if (start == StartPositions.AUDIENCE)
                return startPosAudienceRed;
            else if (start == StartPositions.AWAY)
                return startPosAwayRed;
        }

        return startPosAudienceBlue;

    }


    public static void setAlliance(AllianceInfo temp){
        alliance = temp;
    }

    public static AllianceInfo getAlliance(){
        return alliance;
    }

    public static void setStartIndicator(StartPositions temp){
        start = temp;
    }

    public static StartPositions getStartIndicator() {
        return start;
    }
}
