package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.enums.AllianceInfo;
import org.firstinspires.ftc.teamcode.util.enums.StartPositions;

public class MedJunctionData {
    //GENERIC
//    static Pose2d GENERIC1 = new Pose2d(34, 40, Math.toRadians(180)); //READY
//    static Pose2d GENERIC2 = new Pose2d(34, 16, Math.toRadians(180)); //READY
//    static Pose2d GENERICPark1 = new Pose2d(32, 10, Math.toRadians(180)); //READY
//    static Pose2d GENERICPark2 = new Pose2d(5, 10, Math.toRadians(180)); //READY
//    static Pose2d GENERICPark3 = new Pose2d(60, 10, Math.toRadians(180)); //READY

    //BLUE
    //AUDIENCE
    static Pose2d BlueAudience1 = new Pose2d(-34, 40, Math.toRadians(180)); //READY
    static Pose2d BlueAudience2 = new Pose2d(-34, 18, Math.toRadians(195)); //READY
    static Pose2d BlueAudiencePark1 = new Pose2d(-34, 10, Math.toRadians(180)); //READY
    static Pose2d BlueAudiencePark2 = new Pose2d(-8, 10, Math.toRadians(180)); //READY
    static Pose2d BlueAudiencePark3 = new Pose2d(-55, 10, Math.toRadians(180)); //READY

    //RED
    //AUDIENCE
    static Pose2d RedAudience1 = new Pose2d(-36, -40, Math.toRadians(180)); //READY
    static Pose2d RedAudience2 = new Pose2d(-37, -18, Math.toRadians(168)); //READY
    static Pose2d RedAudiencePark1 = new Pose2d(-37, -10, Math.toRadians(180)); //READY
    static Pose2d RedAudiencePark2 = new Pose2d(-8, -10, Math.toRadians(180)); //READY
    static Pose2d RedAudiencePark3 = new Pose2d(-55, -10, Math.toRadians(180)); //READY

    //BLUE
    //AWAY
//    Is a copy of RedAudience
    static Pose2d BlueAway1 = new Pose2d(36, 40, Math.toRadians(0)); //READY
    static Pose2d BlueAway2 = new Pose2d(34, 18, Math.toRadians(-13)); //READY
    static Pose2d BlueAwayPark1 = new Pose2d(34, 10, Math.toRadians(0)); //READY
    static Pose2d BlueAwayPark2 = new Pose2d(8, 10, Math.toRadians(0)); //READY
    static Pose2d BlueAwayPark3 = new Pose2d(55, 10, Math.toRadians(0)); //READY

    //RED
    //AWAY
    //Is a copy of BlueAudience
    static Pose2d RedAway1 = new Pose2d(36, -40, Math.toRadians(0)); //READY
    static Pose2d RedAway2 = new Pose2d(34, -18, Math.toRadians(13)); //READY
    static Pose2d RedAwayPark1 = new Pose2d(34, -10, Math.toRadians(0)); //READY
    static Pose2d RedAwayPark2 = new Pose2d(8, -10, Math.toRadians(0)); //READY
    static Pose2d RedAwayPark3 = new Pose2d(55, -10, Math.toRadians(0)); //READY

    public static Pose2d getData(int stage, int mode){
        if(Storage.getAlliance() == AllianceInfo.RED && Storage.getStartIndicator() == StartPositions.AUDIENCE){
            switch(stage){
                case 1:
                    return RedAudience1;
                case 2:
                    return RedAudience2;
                case 3:
                    return RedAudiencePark1;
                case 4:
                    return RedAudiencePark2;
                case 5:
                    return RedAudiencePark3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.RED && Storage.getStartIndicator() == StartPositions.AWAY){
            switch(stage){
                case 1:
                    return RedAway1;
                case 2:
                    return RedAway2;
                case 3:
                    return RedAwayPark1;
                case 4:
                    return RedAwayPark2;
                case 5:
                    return RedAwayPark3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.BLUE && Storage.getStartIndicator() == StartPositions.AUDIENCE){
            switch(stage){
                case 1:
                    return BlueAudience1;
                case 2:
                    return BlueAudience2;
                case 3:
                    return BlueAudiencePark1;
                case 4:
                    return BlueAudiencePark2;
                case 5:
                    return BlueAudiencePark3;
            }
        }
        else if(Storage.getAlliance() == AllianceInfo.BLUE && Storage.getStartIndicator() == StartPositions.AWAY){
            switch(stage){
                case 1:
                    return BlueAway1;
                case 2:
                    return BlueAway2;
                case 3:
                    return BlueAwayPark1;
                case 4:
                    return BlueAwayPark2;
                case 5:
                    return BlueAwayPark3;
            }
        }
        return null;
    }
}
