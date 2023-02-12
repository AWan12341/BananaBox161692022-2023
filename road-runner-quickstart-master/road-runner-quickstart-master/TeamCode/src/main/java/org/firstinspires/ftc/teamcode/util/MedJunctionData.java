package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.enums.AllianceInfo;
import org.firstinspires.ftc.teamcode.util.enums.StartPositions;

public class MedJunctionData {
    //BLUE
    //AUDIENCE
    static Pose2d BlueAudience1 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d BlueAudience2 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d BlueAudience3 = new Pose2d(0, 0, Math.toRadians(0));

    //RED
    //AUDIENCE
    static Pose2d RedAudience1 = new Pose2d(-36, -16, Math.toRadians(165)); //READY
    static Pose2d RedAudience2 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d RedAudience3 = new Pose2d(1, 1, Math.toRadians(0));

    //BLUE
    //AWAY
    static Pose2d BlueAway1 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d BlueAway2 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d BlueAway3 = new Pose2d(0, 0, Math.toRadians(0));

    //RED
    //AUDIENCE
    static Pose2d RedAway1 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d RedAway2 = new Pose2d(0, 0, Math.toRadians(0));
    static Pose2d RedAway3 = new Pose2d(0, 0, Math.toRadians(0));

    public static Pose2d getData(int stage){
        if(Storage.getAlliance() == AllianceInfo.RED && Storage.getStartIndicator() == StartPositions.AUDIENCE){
            switch(stage){
                case 1:
                    return RedAudience1;
                case 2:
                    return RedAudience2;
                case 3:
                    return RedAudience3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.RED && Storage.getStartIndicator() == StartPositions.AWAY){
            switch(stage){
                case 1:
                    return RedAway1;
                case 2:
                    return RedAway2;
                case 3:
                    return RedAway3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.BLUE && Storage.getStartIndicator() == StartPositions.AUDIENCE){
            switch(stage){
                case 1:
                    return BlueAudience1;
                case 2:
                    return BlueAudience2;
                case 3:
                    return BlueAudience3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.BLUE && Storage.getStartIndicator() == StartPositions.AWAY){
            switch(stage){
                case 1:
                    return BlueAway1;
                case 2:
                    return BlueAway2;
                case 3:
                    return BlueAway3;
            }
        }
        return null;
    }
}
