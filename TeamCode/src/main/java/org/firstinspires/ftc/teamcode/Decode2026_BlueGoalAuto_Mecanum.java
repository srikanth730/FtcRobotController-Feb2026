package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * Decode 2026 autonomous (relative moves) + launcher logic pulled from your TeleOp.
 *
 * Drive hardware names:
 *   left_front_drive, right_front_drive, left_back_drive, right_back_drive
 *
 * Launcher hardware names (from your TeleOp):
 *   left_launcher, right_launcher, left_feeder, right_feeder, diverter
 *
 * IMU name:
 *   imu
 */

@Autonomous(name = "Decode 2026: Blue Goal Auto + Launch", group = "Robot")
public class Decode2026_BlueGoalAuto_Mecanum extends LinearOpMode {

    // -------------------- DRIVE --------------------
    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;

    private IMU imu = null;

    private double headingError  = 0.0;
    private double targetHeading = 0.0;

    private double driveSpeed  = 0.0;
    private double strafeSpeed = 0.0;
    private double turnSpeed   = 0.0;

    private double lfPower = 0.0;
    private double rfPower = 0.0;
    private double lbPower = 0.0;
    private double rbPower = 0.0;

    private int lfTarget = 0;
    private int rfTarget = 0;
    private int lbTarget = 0;
    private int rbTarget = 0;

    // -------------------- LAUNCHER (from your TeleOp) --------------------
    private DcMotorEx leftLauncher  = null;
    private DcMotorEx rightLauncher = null;

    private CRServo leftFeeder  = null;
    private CRServo rightFeeder = null;

    private Servo diverter = null;

    private static final double FEED_TIME_SECONDS = 0.80;
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;

    private static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; // ticks/sec
    private static final double LAUNCHER_CLOSE_MIN_VELOCITY    = 1175;

    private static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350;
    private static final double LAUNCHER_FAR_MIN_VELOCITY    = 1325;

    // Choose CLOSE or FAR for this auto
    private double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    private double launcherMin    = LAUNCHER_CLOSE_MIN_VELOCITY;

    private static final double LEFT_POSITION  = 0.2962;
    private static final double RIGHT_POSITION = 0.0;

    // -------------------- YOUR HARDWARE CONSTANTS --------------------
    static final double COUNTS_PER_MOTOR_REV  = 537.7;          // Yellow Jacket 312RPM (typical)
    static final double DRIVE_GEAR_REDUCTION  = 1.0;            // external gearing (change if needed)
    static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;   // Ø104mm GripForce mecanum
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // -------------------- TUNABLES --------------------
    static final double DRIVE_SPEED  = 0.40;
    static final double STRAFE_SPEED = 0.35;
    static final double TURN_SPEED   = 0.22;

    static final double HEADING_THRESHOLD_DEG = 1.0;

    static final double P_TURN_GAIN  = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    // -------------------- ROUTE PARAMETERS --------------------
    // Step 2 distance not provided by you; measure on-field and set this.
    static final double BACKUP_TO_TRIANGLE_APEX_INCHES = 24.0; // TODO: measure & update

    static final double HEADING_GOAL_DEG   = 0.0;
    static final double HEADING_LEFT45_DEG = 45.0;

    @Override
    public void runOpMode() {

        // -------------------- MAP HARDWARE --------------------
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        leftFeeder    = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder   = hardwareMap.get(CRServo.class, "right_feeder");
        diverter      = hardwareMap.get(Servo.class, "diverter");

        // -------------------- MOTOR DIRECTIONS (as you specified) --------------------
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher directions (from your TeleOp)
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightLauncher default FORWARD

        // Feeders (from your TeleOp)
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------------------- IMU INIT --------------------
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection   = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // -------------------- MODES / BRAKE --------------------
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PIDF (from your TeleOp)
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Initialize feeder outputs
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        // Diverter initial position (from your TeleOp)
        diverter.setPosition(LEFT_POSITION);

        // -------------------- WAIT FOR START --------------------
        while (opModeInInit()) {
            telemetry.addData("Heading (deg)", "%.1f", getHeading());
            telemetry.addData("Counts/Inch", "%.2f", COUNTS_PER_INCH);
            telemetry.addData("Step2 back inches", "%.1f", BACKUP_TO_TRIANGLE_APEX_INCHES);
            telemetry.addData("Launcher tgt/min", "%.0f / %.0f", launcherTarget, launcherMin);
            telemetry.update();
        }

        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        // -------------------- AUTON STEPS --------------------
        // 1) Start pose described (no code)

        // 2) Back up to apex of big triangle, no angle change
        driveStraight(DRIVE_SPEED, -BACKUP_TO_TRIANGLE_APEX_INCHES, HEADING_GOAL_DEG);

        // 3) Shoot 3 artifacts, wait 3 seconds total (shooting function includes timing)
        shoot3ArtifactsAndWait3s();

        // 4) Move forward 6 inches
        driveStraight(DRIVE_SPEED, 6.0, HEADING_GOAL_DEG);

        // 5) Turn left 45 degrees
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        // 6) Move forward 20 inches (intake collects)
        driveStraight(DRIVE_SPEED, 20.0, HEADING_LEFT45_DEG);

        // 7) Return to step2 pose facing goal
        driveStraight(DRIVE_SPEED, -20.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);
        driveStraight(DRIVE_SPEED, -6.0, HEADING_GOAL_DEG);

        // 8) Shoot 3 artifacts
        shoot3ArtifactsAndWait3s();

        // 9) Turn left 45 degrees
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        // 10) Strafe left 20 inches
        strafeDistance(STRAFE_SPEED, -20.0, HEADING_LEFT45_DEG);

        // 11) Move forward 20 inches
        driveStraight(DRIVE_SPEED, 20.0, HEADING_LEFT45_DEG);

        // 12) Return to step2 pose facing goal
        driveStraight(DRIVE_SPEED, -20.0, HEADING_LEFT45_DEG);
        strafeDistance(STRAFE_SPEED, 20.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);

        // 13) Shoot 3 artifacts
        shoot3ArtifactsAndWait3s();

        // 14) Turn left 45 degrees
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        // 15) Move forward 20 inches
        driveStraight(DRIVE_SPEED, 20.0, HEADING_LEFT45_DEG);

        // 16) Move backward 20 inches
        driveStraight(DRIVE_SPEED, -20.0, HEADING_LEFT45_DEG);

        // 17) Strafe left 26 inches
        strafeDistance(STRAFE_SPEED, -26.0, HEADING_LEFT45_DEG);

        // 18) Move forward 20 inches
        driveStraight(DRIVE_SPEED, 20.0, HEADING_LEFT45_DEG);

        // 19) Return to step2 pose facing goal
        driveStraight(DRIVE_SPEED, -20.0, HEADING_LEFT45_DEG);
        strafeDistance(STRAFE_SPEED, 26.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);

        // 20) Shoot 3 artifacts
        shoot3ArtifactsAndWait3s();

        // 21) Strafe left 20 inches
        strafeDistance(STRAFE_SPEED, -20.0, HEADING_GOAL_DEG);

        // Stop everything
        moveRobot(0.0, 0.0, 0.0);
        stopLauncher();

        telemetry.addData("Auto", "Complete");
        telemetry.update();
        sleep(500);
    }

    // =========================================================================================
    // LAUNCHER: Auto-friendly "shoot 3" based on your TeleOp behavior
    // =========================================================================================

    private void shoot3ArtifactsAndWait3s() {
        // (Empty line placeholder you asked for; keep for custom actions)

        // Spin up and fire 3 shots using RIGHT feeder (matches your TeleOp usage)
        boolean spunUp = spinUpLauncher(1.25); // seconds timeout to reach min velocity

        if (spunUp && opModeIsActive()) {
            fireOneShotRight();
            fireOneShotRight();
            fireOneShotRight();
        } else {
            // If we didn't reach speed, still wait to avoid sequence timing chaos
            // (You can change this behavior if you prefer to skip shots)
        }

        // You requested: "wait for 3 seconds to finish the shooting"
        sleep(3000);

        // Optional: stop flywheels after each volley (comment out if you want them to stay spinning)
        stopLauncher();
    }

    private boolean spinUpLauncher(double timeoutSeconds) {
        leftLauncher.setVelocity(launcherTarget);
        rightLauncher.setVelocity(launcherTarget);

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < (long) (timeoutSeconds * 1000.0)) {
            if (rightLauncher.getVelocity() >= launcherMin) return true;
            telemetry.addData("Launcher", "Spinning up...");
            telemetry.addData("Vel L/R", "%.0f / %.0f", leftLauncher.getVelocity(), rightLauncher.getVelocity());
            telemetry.update();
            sleep(10);
        }
        return rightLauncher.getVelocity() >= launcherMin;
    }

    private void fireOneShotRight() {
        if (!opModeIsActive()) return;

        rightFeeder.setPower(FULL_SPEED);
        sleep((long) (FEED_TIME_SECONDS * 1000.0));
        rightFeeder.setPower(STOP_SPEED);

        // Small settle time between shots (helps reduce double-feeds)
        sleep(150);
    }

    private void stopLauncher() {
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    // =========================================================================================
    // DRIVE: High level movement
    // =========================================================================================

    public void driveStraight(double maxDriveSpeed, double distanceInches, double headingDeg) {
        if (!opModeIsActive()) return;

        int moveCounts = (int) Math.round(distanceInches * COUNTS_PER_INCH);

        lfTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        rfTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
        lbTarget = leftBackDrive.getCurrentPosition() + moveCounts;
        rbTarget = rightBackDrive.getCurrentPosition() + moveCounts;

        setAllTargets(lfTarget, rfTarget, lbTarget, rbTarget);
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0.0, 0.0);

        while (opModeIsActive() && anyBusy()) {
            double correction = getSteeringCorrection(headingDeg, P_DRIVE_GAIN);

            if (distanceInches < 0) correction *= -1.0;

            correction = Range.clip(correction, -0.35, 0.35);

            moveRobot(maxDriveSpeed, 0.0, correction);
            sendTelemetry("Drive", true);
        }

        moveRobot(0.0, 0.0, 0.0);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeDistance(double maxStrafeSpeed, double distanceInches, double headingDeg) {
        if (!opModeIsActive()) return;

        int moveCounts = (int) Math.round(distanceInches * COUNTS_PER_INCH);

        lfTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        rfTarget = rightFrontDrive.getCurrentPosition() - moveCounts;
        lbTarget = leftBackDrive.getCurrentPosition() - moveCounts;
        rbTarget = rightBackDrive.getCurrentPosition() + moveCounts;

        setAllTargets(lfTarget, rfTarget, lbTarget, rbTarget);
        setAllModes(DcMotor.RunMode.RUN_TO_POSITION);

        maxStrafeSpeed = Math.abs(maxStrafeSpeed);
        moveRobot(0.0, maxStrafeSpeed, 0.0);

        while (opModeIsActive() && anyBusy()) {
            double correction = getSteeringCorrection(headingDeg, P_DRIVE_GAIN);
            correction = Range.clip(correction, -0.35, 0.35);

            moveRobot(0.0, maxStrafeSpeed, correction);
            sendTelemetry("Strafe", true);
        }

        moveRobot(0.0, 0.0, 0.0);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToHeading(double maxTurnSpeed, double headingDeg) {
        getSteeringCorrection(headingDeg, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD_DEG)) {
            double turn = getSteeringCorrection(headingDeg, P_TURN_GAIN);
            turn = Range.clip(turn, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0.0, 0.0, turn);
            sendTelemetry("Turn", false);
        }

        moveRobot(0.0, 0.0, 0.0);
    }

    // =========================================================================================
    // DRIVE: Low level helpers
    // =========================================================================================

    public double getSteeringCorrection(double desiredHeadingDeg, double proportionalGain) {
        targetHeading = desiredHeadingDeg;

        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    public void moveRobot(double drive, double strafe, double turn) {
        driveSpeed  = drive;
        strafeSpeed = strafe;
        turnSpeed   = turn;

        double lf = drive + strafe - turn;
        double rf = drive - strafe + turn;
        double lb = drive - strafe - turn;
        double rb = drive + strafe + turn;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        lfPower = lf; rfPower = rf; lbPower = lb; rbPower = rb;

        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lbPower);
        rightBackDrive.setPower(rbPower);
    }

    private void setAllTargets(int lf, int rf, int lb, int rb) {
        leftFrontDrive.setTargetPosition(lf);
        rightFrontDrive.setTargetPosition(rf);
        leftBackDrive.setTargetPosition(lb);
        rightBackDrive.setTargetPosition(rb);
    }

    private void setAllModes(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    private void setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    private boolean anyBusy() {
        return leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy()  || rightBackDrive.isBusy();
    }

    private void sendTelemetry(String motion, boolean showEncoders) {
        telemetry.addData("Motion", motion);
        telemetry.addData("Heading tgt/cur", "%.1f / %.1f", targetHeading, getHeading());
        telemetry.addData("Heading err", "%.1f", headingError);
        telemetry.addData("Drive/Strafe/Turn", "%.2f / %.2f / %.2f", driveSpeed, strafeSpeed, turnSpeed);

        if (showEncoders) {
            telemetry.addData("Tgt LF RF", "%7d %7d", lfTarget, rfTarget);
            telemetry.addData("Tgt LB RB", "%7d %7d", lbTarget, rbTarget);
            telemetry.addData("Pos LF RF", "%7d %7d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Pos LB RB", "%7d %7d",
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        }

        telemetry.addData("Launcher vel L/R", "%.0f / %.0f", leftLauncher.getVelocity(), rightLauncher.getVelocity());
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}




