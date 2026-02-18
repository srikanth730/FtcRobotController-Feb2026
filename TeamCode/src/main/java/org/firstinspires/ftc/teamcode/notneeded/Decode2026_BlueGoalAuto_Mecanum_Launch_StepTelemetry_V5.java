package org.firstinspires.ftc.teamcode.notneeded;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "Decode 2026: Blue Goal Auto V5 (0.8 + Intake)", group = "Robot")
@Disabled
public class Decode2026_BlueGoalAuto_Mecanum_Launch_StepTelemetry_V5 extends LinearOpMode {

    // -------------------- DRIVE --------------------
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private IMU imu;

    private double headingError = 0.0;
    private double targetHeading = 0.0;

    private int lfTarget = 0, rfTarget = 0, lbTarget = 0, rbTarget = 0;

    // -------------------- INTAKE (from your TeleOp) --------------------
    private DcMotor intake = null;
    private static final double INTAKE_ON_POWER = 1.00;
    private static final double INTAKE_OFF_POWER = 0.0;
    private boolean intakeOn = false;

    // -------------------- LAUNCHER (from your TeleOp) --------------------
    private DcMotorEx leftLauncher, rightLauncher;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;

    private static final double FEED_TIME_SECONDS = 0.80;
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;

    private static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; // ticks/sec
    private static final double LAUNCHER_CLOSE_MIN_VELOCITY    = 1175;

    private double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    private double launcherMin    = LAUNCHER_CLOSE_MIN_VELOCITY;

    private static final double LEFT_POSITION  = 0.2962;

    // -------------------- COUNTS/INCH --------------------
    static final double COUNTS_PER_MOTOR_REV  = 537.7;
    static final double DRIVE_GEAR_REDUCTION  = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 104.0 / 25.4;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // -------------------- TUNABLES --------------------
    static final double DRIVE_SPEED  = 0.80; // per your request
    static final double STRAFE_SPEED = 0.60;
    static final double TURN_SPEED   = 0.25;

    static final double HEADING_THRESHOLD_DEG = 1.0;
    static final double P_TURN_GAIN  = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    static final double MAX_CORRECTION = 0.28;

    // -------------------- HEADINGS --------------------
    static final double HEADING_GOAL_DEG   = 0.0;
    static final double HEADING_LEFT45_DEG = 45.0;

    // -------------------- STEP COUNTER --------------------
    private int stepNum = 0;

    @Override
    public void runOpMode() {

        // -------------------- MAP HARDWARE --------------------
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake        = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder    = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder   = hardwareMap.get(CRServo.class, "right_feeder");
        diverter      = hardwareMap.get(Servo.class, "diverter");

        // Drive directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Launcher + intake directions (from your TeleOp)
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Feeder direction (from your TeleOp)
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU init (edit if hub orientation differs)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection   = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        // Encoders + brake
        setAllModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setIntake(false);

        // Launcher config
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        diverter.setPosition(LEFT_POSITION);

        while (opModeInInit()) {
            telemetry.addLine("Ready: Decode 2026 Blue Goal Auto V4");
            telemetry.addData("Heading (deg)", "%.1f", getHeading());
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.addData("Counts/Inch", "%.2f", COUNTS_PER_INCH);
            telemetry.addData("Drive/Strafe/Turn", "%.2f / %.2f / %.2f", DRIVE_SPEED, STRAFE_SPEED, TURN_SPEED);
            telemetry.update();
        }

        imu.resetYaw();

        // ==================== AUTON STEPS ====================

        step(1, "Start: touching blue goal (top-left), 45°");

        step(2, "Back 52 in, hold 0°");
        driveStraight(DRIVE_SPEED, -52.0, HEADING_GOAL_DEG);

        step(3, "Shoot 3, wait 3s");
        shoot3ArtifactsAndWait3s();

        step(4, "Turn left to 45°");
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        // Intake BEFORE step 6, OFF after step 6
        step(6, "Intake ON; forward 28 in, hold 45°; intake OFF");
        setIntake(true);
        driveStraight(DRIVE_SPEED, 28.0, HEADING_LEFT45_DEG);
        setIntake(false);

        step(7, "Back 28; turn to 0° (return step2 pose)");
        driveStraight(DRIVE_SPEED, -28.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);

        step(8, "Shoot 3, wait 3s");
        shoot3ArtifactsAndWait3s();

        step(9, "Turn left to 45°");
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        step(10, "Strafe left 24 in, hold 45°");
        strafeDistance(STRAFE_SPEED, -24.0, HEADING_LEFT45_DEG);

        // Intake BEFORE step 11, OFF after step 11
        step(11, "Intake ON; forward 28 in; intake OFF");
        setIntake(true);
        driveStraight(DRIVE_SPEED, 28.0, HEADING_LEFT45_DEG);
        setIntake(false);

        step(12, "Back 28; strafe right 24; turn 0° (return step2 pose)");
        driveStraight(DRIVE_SPEED, -28.0, HEADING_LEFT45_DEG);
        strafeDistance(STRAFE_SPEED, 24.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);

        step(13, "Shoot 3, wait 3s");
        shoot3ArtifactsAndWait3s();

        step(14, "Turn left to 45°");
        turnToHeading(TURN_SPEED, HEADING_LEFT45_DEG);

        step(15, "Strafe left 36 in, hold 45°");
        strafeDistance(STRAFE_SPEED, -36.0, HEADING_LEFT45_DEG);

        // Intake BEFORE step 16, OFF after step 16
        step(16, "Intake ON; forward 28 in; intake OFF");
        setIntake(true);
        driveStraight(DRIVE_SPEED, 28.0, HEADING_LEFT45_DEG);
        setIntake(false);

        step(17, "Return step2 pose: back 28; strafe right 36; turn 0°");
        driveStraight(DRIVE_SPEED, -28.0, HEADING_LEFT45_DEG);
        strafeDistance(STRAFE_SPEED, 36.0, HEADING_LEFT45_DEG);
        turnToHeading(TURN_SPEED, HEADING_GOAL_DEG);

        step(18, "Shoot 3, wait 3s");
        shoot3ArtifactsAndWait3s();

        step(19, "Strafe left 20 in, hold 0°");
        strafeDistance(STRAFE_SPEED, -20.0, HEADING_GOAL_DEG);

        // Stop everything
        moveRobot(0.0, 0.0, 0.0);
        setIntake(false);
        stopLauncher();

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
        sleep(500);
    }

    // =========================================================================================
    // INTAKE HELPERS (from your TeleOp behavior)
    // =========================================================================================
    private void setIntake(boolean on) {
        intakeOn = on;
        intake.setPower(on ? INTAKE_ON_POWER : INTAKE_OFF_POWER);

        //telemetry.addData("STEP %d Intake", stepNum, on ? "ON" : "OFF");
        telemetry.addData("Intake Power", "%.2f", on ? INTAKE_ON_POWER : INTAKE_OFF_POWER);
        telemetry.update();
        sleep(60);
    }

    // =========================================================================================
    // STEP TELEMETRY
    // =========================================================================================
    private void step(int number, String description) {
        stepNum = number;
        telemetry.addLine("--------------------------------");
        telemetry.addData("STEP", "%d", stepNum);
        telemetry.addData("Action", "%s", description);
        telemetry.addData("Heading (deg)", "%.1f", getHeading());
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.update();
        sleep(150);
    }

    // =========================================================================================
    // LAUNCHER: Shoot 3 + wait 3 seconds
    // =========================================================================================
    private void shoot3ArtifactsAndWait3s() {
        boolean spunUp = spinUpLauncher(1.25);

        if (spunUp && opModeIsActive()) {
            fireOneShotRight();
            fireOneShotRight();
            fireOneShotRight();
        }

        sleep(3000);
        stopLauncher();
    }

    private boolean spinUpLauncher(double timeoutSeconds) {
        leftLauncher.setVelocity(launcherTarget);
        rightLauncher.setVelocity(launcherTarget);

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < (long) (timeoutSeconds * 1000.0)) {
            if (rightLauncher.getVelocity() >= launcherMin) return true;

            //telemetry.addData("STEP %d Launcher", stepNum, "Spin-up");
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
        sleep(150);
    }

    private void stopLauncher() {
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    // =========================================================================================
    // DRIVE: Encoder moves + IMU heading hold
    // =========================================================================================
    private void driveStraight(double maxDriveSpeed, double distanceInches, double headingDeg) {
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
            correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);
            moveRobot(maxDriveSpeed, 0.0, correction);
        }

        moveRobot(0.0, 0.0, 0.0);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeDistance(double maxStrafeSpeed, double distanceInches, double headingDeg) {
        if (!opModeIsActive()) return;

        int moveCounts = (int) Math.round(distanceInches * COUNTS_PER_INCH);

        // Strafe RIGHT (+): LF+, RF-, LB-, RB+
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
            correction = Range.clip(correction, -MAX_CORRECTION, MAX_CORRECTION);
            moveRobot(0.0, maxStrafeSpeed, correction);
        }

        moveRobot(0.0, 0.0, 0.0);
        setAllModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnToHeading(double maxTurnSpeed, double headingDeg) {
        getSteeringCorrection(headingDeg, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD_DEG)) {
            double turn = getSteeringCorrection(headingDeg, P_TURN_GAIN);
            turn = Range.clip(turn, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0.0, 0.0, turn);
        }
        moveRobot(0.0, 0.0, 0.0);
    }

    // =========================================================================================
    // DRIVE: Helpers
    // =========================================================================================
    private double getSteeringCorrection(double desiredHeadingDeg, double proportionalGain) {
        targetHeading = desiredHeadingDeg;
        headingError = targetHeading - getHeading();

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1.0, 1.0);
    }

    private void moveRobot(double drive, double strafe, double turn) {
        double lf = drive + strafe - turn;
        double rf = drive - strafe + turn;
        double lb = drive - strafe + turn;
        double rb = drive + strafe - turn;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
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
                leftBackDrive.isBusy() || rightBackDrive.isBusy();
    }

    private double getHeading() {
        YawPitchRollAngles o = imu.getRobotYawPitchRollAngles();
        return o.getYaw(AngleUnit.DEGREES);
    }
}




