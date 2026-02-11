package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
public class SampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION TO END POSITION
        // DRIVE MOVEMENT STATE
        // SHOOT ATTEMPT TO SCORE POINT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD


    }

    private final Pose moveBackPose = new Pose(20.63037667071689, 121.98128797083845, Math.toRadians(138));
    private final Pose shootPose = new Pose(20.63037667071689, 121.98128797083845, Math.toRadians(138));

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }


}
