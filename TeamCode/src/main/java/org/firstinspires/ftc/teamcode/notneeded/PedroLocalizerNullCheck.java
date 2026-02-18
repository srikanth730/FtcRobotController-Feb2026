package org.firstinspires.ftc.teamcode.notneeded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Field;

@Autonomous(name="Pedro Localizer Null Check", group="Test")
@Disabled
public class PedroLocalizerNullCheck extends LinearOpMode {

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);

        boolean hasLocalizer = hasNonNullLocalizerField(follower);
        telemetry.addData("Follower has Localizer?", hasLocalizer);
        telemetry.update();

        waitForStart();

        if (!hasLocalizer) {
            telemetry.addLine("Fix: configure Localizer in FollowerBuilder/constants.");
            telemetry.update();
            sleep(3000);
            return;
        }

        // Safe only if Localizer exists

    }

    private boolean hasNonNullLocalizerField(Follower follower) {
        try {
            Field f = follower.getClass().getDeclaredField("localizer");
            f.setAccessible(true);
            return f.get(follower) != null;
        } catch (Exception e) {
            // Field name may differ by version; treat as unknown
            return false;
        }
    }
}
