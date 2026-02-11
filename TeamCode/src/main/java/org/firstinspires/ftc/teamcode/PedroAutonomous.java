
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous 31567", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
     ///   pathState = autonomousPathUpdate(); // Update autonomous state machine
        autonomousPathUpdate();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain moveback;
        public PathChain movefwdabit;
        public PathChain collectfirststack;
        public PathChain movebacktoline1rev;
        public PathChain movebackwardtoshoot;
        public PathChain starttosecondstack;
        public PathChain collectsecondstack;
        public PathChain movebacktoline2rev;
        public PathChain movebackwardtoshoot2;
        public PathChain starttogate;
        public PathChain openthegate;
        public PathChain starttothirdstack1;
        public PathChain starttothirdstack2;
        public PathChain collectthirdstack;
        public PathChain movebacktoline3rev;
        public PathChain movebackwardtoshoot3;
        public PathChain strafeleft;

        public Paths(Follower follower) {
            moveback = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.630, 121.981),

                                    new Pose(62.464, 80.836)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138))

                    .build();

            movefwdabit = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.464, 80.836),

                                    new Pose(59.592, 83.953)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            collectfirststack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.592, 83.953),

                                    new Pose(18.920, 83.706)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            movebacktoline1rev = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.920, 83.706),

                                    new Pose(59.437, 84.081)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            movebackwardtoshoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.437, 84.081),

                                    new Pose(62.057, 81.190)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            starttosecondstack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.057, 81.190),

                                    new Pose(51.792, 59.442)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            collectsecondstack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(51.792, 59.442),

                                    new Pose(18.818, 60.078)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            movebacktoline2rev = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.818, 60.078),

                                    new Pose(59.519, 84.039)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            movebackwardtoshoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.519, 84.039),

                                    new Pose(62.052, 81.091)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            starttogate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.052, 81.091),

                                    new Pose(61.662, 70.247)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            openthegate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.662, 70.247),

                                    new Pose(13.260, 69.948)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            starttothirdstack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.260, 69.948),

                                    new Pose(61.221, 49.104)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            starttothirdstack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.221, 49.104),

                                    new Pose(60.844, 35.948)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            collectthirdstack = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.844, 35.948),

                                    new Pose(18.727, 35.364)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            movebacktoline3rev = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.727, 35.364),

                                    new Pose(59.623, 84.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            movebackwardtoshoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.623, 84.000),

                                    new Pose(62.338, 81.260)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            strafeleft = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.338, 81.260),

                                    new Pose(49.143, 64.455)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}
