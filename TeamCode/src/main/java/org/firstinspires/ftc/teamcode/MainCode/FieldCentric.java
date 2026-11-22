package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
/*
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

 */

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "FieldCentric", group = "Test")

public class FieldCentric extends LinearOpMode {

    // Drive motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearRight = null;
    private DcMotorEx outerIntake = null;
    private DcMotorEx innerIntake = null;
    private DcMotorEx turretSpin = null;
    private DcMotorEx turretShoot = null;
    private SparkFunOTOS odometry = null;

    private static final double VELCONST = 6000 / 60.0 * 28;
    // remove negative when it has a turret
    private static final double TURRETGEARRATIO = -3.61 * 3.61 * 3.61 * 28 * 7.6667 / 3;
    private double lowPower;
    private double outerIntakeVel;
    private double innerIntakeVel;
    private boolean manualTurret;
    private boolean activeLauncher;
    private int activeIntake; // 0 = off, 1 = in, 2 = out
    private double prevBotHeading;

    /* LIMELIGHT
    // Index of the Python pipeline in the Limelight web UI.
    private static final int CLUSTER_PIPELINE = 1;

    // Camera pitch in degrees, + = tilted up (0 = level, 90 = straight up).
    // Measure this on the robot.
    private static final double CAMERA_PITCH_DEG = 30.0;

    // Python input [1] values
    private static final double ANY = 0, BLUE = 1, RED = 2;

    // llpython layout (must match apriltag_cluster.py). Distances are in inches,
    // angles in degrees: use Math.toRadians() before Math.sin/cos/tan.
    private static final int CLUSTER_ID = 0, NUM_TAGS = 1, RANGE = 2, BEARING = 3,
            HEIGHT = 4, ELEVATION = 5, STICKER_PITCH = 6, ROLL = 7;

    private static double alliance;
    */

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontleft");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearright");

        // Intake motors
        outerIntake = hardwareMap.get(DcMotorEx.class, "intake1");
        innerIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        // Turret motors
        turretSpin = hardwareMap.get(DcMotorEx.class, "turretspin");
        turretShoot = hardwareMap.get(DcMotorEx.class, "turretshoot");

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        turretSpin.setDirection(DcMotor.Direction.REVERSE);
        turretShoot.setDirection(DcMotor.Direction.REVERSE);
        innerIntake.setDirection(DcMotor.Direction.REVERSE);

        // motor modes
        // assumes initial position is straight forwards
        turretSpin.setTargetPosition(0);
        turretSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretSpin.setVelocity(0);

        // Initialize IMU and odometry
        IMU imu = hardwareMap.get(IMU.class, "imu");

        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        odometry.setLinearUnit(DistanceUnit.METER);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        // all of these below are temp values that need to be changed
        odometry.setOffset(new SparkFunOTOS.Pose2D(0.1885,0.035,180));
        odometry.setLinearScalar(9.467001327014218);
        odometry.setAngularScalar(1.0266259166023104);
        odometry.resetTracking();

        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        lowPower = 1;
        outerIntakeVel = 0.0;
        innerIntakeVel = 0.0;
        manualTurret = true;
        activeLauncher = false;
        activeIntake = 0;
        prevBotHeading = 0.0;

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                )
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        /*
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(CLUSTER_PIPELINE);
        limelight.start();

        // LIMELIGHT
        while (opModeInInit()) {
            if (gamepad1.b) alliance = RED;
            if (gamepad1.x) alliance = BLUE;
            if (gamepad1.a) alliance = ANY;
            telemetry.addData("Alliance (B red / X blue / A any)",
                    alliance == RED ? "RED" : alliance == BLUE ? "BLUE" : "ANY");
            telemetry.update();
        }
        // Inputs: [0] 1 = inputs valid (0 makes the script use its own defaults),
        // [1] alliance, [2] camera pitch in degrees
        limelight.updatePythonInputs(new double[] {1, alliance, CAMERA_PITCH_DEG, 0, 0, 0, 0, 0});

         */

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Joystick controls
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Get robot heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            /*
            // LIMELIGHT
            LLResult result = limelight.getLatestResult();
            double[] py = (result != null) ? result.getPythonOutput() : null;
             */
            double[] py = {};

            // saving new target imu pos if rotated
            // boolean hasRotated;
            // if (Math.abs(rx) > 0.05) prevBotHeading = botHeading;

            // intake
            if (gamepad1.aWasPressed()) {
                if (activeIntake == 1) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = VELCONST * 0.6;
                    activeIntake = 1;
                }

            } else if (gamepad1.bWasPressed()) {
                if (activeIntake == 2) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = -VELCONST * 0.6;
                    activeIntake = 2;
                }

            }

            if (gamepad2.b) {
                innerIntakeVel = VELCONST * 0.75;
            }
            else if (gamepad2.a) {
                innerIntakeVel = -VELCONST * 0.75;
            }

            if (gamepad1.right_bumper){
                lowPower = 0.5;
            }
            else lowPower = 1;
            y *= lowPower;
            x *= lowPower;
            rx *= lowPower;

            if (gamepad1.leftBumperWasPressed()){
                imu.resetYaw();
                odometry.setPosition(new SparkFunOTOS.Pose2D(1.0266,0.02,180));
            }

            // launcher
            if (gamepad2.dpadUpWasPressed()) {
                activeLauncher  = !activeLauncher;
            }

            // if (gamepad2.aWasPressed()) manualTurret = !manualTurret;

            /*
            Important numbers to aim
            the center of an upwards hive is 0.5686 m out, 0.32375 m across, and 1.51257 m up
            from the center of the field

            assume position starts from the left corner
             */

            SparkFunOTOS.Pose2D odometryPos = odometry.getPosition();

            // using limelight if it detects a cluster
            if (py != null && py[0] != -1) {
                if (py[0] == 30 || py[0] == 42) {
                    odometryPos = new SparkFunOTOS.Pose2D(1.5575 - py[2] * Math.cos(py[3]), 1.50625 - py[2] * Math.sin(py[3]), odometryPos.h);
                }
                else odometryPos = new SparkFunOTOS.Pose2D(2.1025 - py[2] * Math.cos(py[3]), 1.50625 - py[2] * Math.sin(py[3]), odometryPos.h);
                odometry.setPosition(odometryPos);
            }

            // checking which hive to aim for
            double xDistToTarget;
            if (odometryPos.x < 1.83) xDistToTarget = 1.5575 - odometryPos.x;
            else xDistToTarget = 2.1025 - odometryPos.x;

            double yDistToTarget = 1.50625 - odometryPos.y;

            // specifically amount to rotate, takes into account robot rotation
            double angleToTarget = (Math.atan2(xDistToTarget, yDistToTarget) + botHeading); // in radians
            // Convert angle from -pi to pi to encoder ticks
            int turretTicksToRotate = (int) (angleToTarget * TURRETGEARRATIO / Math.PI);

            // checking if it will rotate outside of the range
            /*
            if (turretTicksToRotate > 0.777 * TURRETGEARRATIO) {
                // setting turret position to the closest point outside the limits
                // there will be some cursed behavior with setting directly to -pi position if the robot is rotated, fix later
                if (turretTicksToRotate > 0.166 * TURRETGEARRATIO) turretTicksToRotate = (int) (-1 * TURRETGEARRATIO);
                else turretTicksToRotate = (int) (0.777 * TURRETGEARRATIO);
            }
            else if (turretTicksToRotate < -1 * TURRETGEARRATIO) {
                // same cursed behavior here too with setting directly instead of math
                // maybe try Math.clamp()?
                turretTicksToRotate = (int) (0.777 * TURRETGEARRATIO);
            }
             */

            // prev guess of launcherVel, will not work
            /*
            double launchAngle = Math.toRadians(45);
            // speed is only the speed of the ball, not the launcher vel
            double launchSpeed = Math.sqrt(Math.abs((4.7 * xDistToTarget * xDistToTarget) /
                    (Math.cos(launchAngle) * Math.cos(launchAngle) * (xDistToTarget * Math.tan(launchAngle) - yDistToTarget))));
            double launcherVel = activeLauncher * launchSpeed * DriveConstants.TICKS_PER_REV / (.045 * 2 * Math.PI); // .045 is radius of wheel
             */

            // interpolating to get launch vel
            double launcherVel = 0;

            if (activeLauncher && !manualTurret) {
                // use interpolation equation here
                // launcherVel = ;
            }


            if (manualTurret) {
                if (gamepad2.dpadRightWasPressed()) {
                    turretTicksToRotate = turretSpin.getCurrentPosition() + 20;
                }
                else if (gamepad2.dpadLeftWasPressed()) {
                    turretTicksToRotate = turretSpin.getCurrentPosition() - 20;
                }
                if (gamepad2.left_bumper) {
                    launcherVel = 1350;
                }
                else if (gamepad2.right_bumper) {
                    launcherVel = 1200;
                }
                else launcherVel = 0;
            }

            // Convert robot-centric controls to field-centric
            double rotX = x * Math.cos(-botHeading)
                    - y * Math.sin(-botHeading);

            double rotY = x * Math.sin(-botHeading)
                    + y * Math.cos(-botHeading);

            // Compensate for imperfect strafing
            rotX *= 1.1;

            // adjust for unintentional rotation

            // Calculate mecanum velocities
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftVel = VELCONST * (rotY + rotX + rx) / denominator;
            double rearLeftVel = VELCONST * (rotY - rotX + rx) / denominator;
            double frontRightVel = VELCONST * (rotY - rotX - rx) / denominator;
            double rearRightVel = VELCONST * (rotY + rotX - rx) / denominator;

            // correcting for misaligned imu movements
            /*
            frontLeftVel += Math.toDegrees(prevBotHeading - botHeading) * VELCONST / 50;
            frontRightVel -= Math.toDegrees(prevBotHeading - botHeading) * VELCONST / 50;
            // accounting for a velocity greater than the max by reducing the opposite wheel instead
            if (frontLeftVel > VELCONST) {
                rearRightVel -= frontLeftVel - VELCONST;
                frontLeftVel = VELCONST;
            }
            if (frontRightVel < -VELCONST) {
                rearLeftVel += frontRightVel + VELCONST;
                frontRightVel = -VELCONST;
            }
             */


            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

            outerIntake.setVelocity(outerIntakeVel);
            innerIntake.setVelocity(innerIntakeVel);

            turretSpin.setTargetPosition(0);
            turretSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // might want pid for this later instead
            turretSpin.setVelocity(0.25 * VELCONST);

            turretShoot.setVelocity(launcherVel);

            // Telemetry
            /*
            telemetry.addData(
                    "Heading",
                    "%.1f°",
                    Math.toDegrees(botHeading)
            );

            telemetry.addData(
                    "Front Left/Right",
                    "%.2f, %.2f",
                    frontLeftVel,
                    frontRightVel
            );

            telemetry.addData(
                    "Rear Left/Right",
                    "%.2f, %.2f",
                    rearLeftVel,
                    rearRightVel
            );
             */
            // Limelight
            /*
            if (py != null && py.length >= 8 && py[CLUSTER_ID] >= 0) {
                telemetry.addData("Cluster", "%d  (%d/4 tags)", (int) py[CLUSTER_ID], (int) py[NUM_TAGS]);
                telemetry.addData("Range (in)", "%.1f", py[RANGE]);
                telemetry.addData("Bearing (+ left)", "%.2f°", py[BEARING]);
                telemetry.addData("Height (in)", "%.1f", py[HEIGHT]);
                telemetry.addData("Elevation", "%.2f°", py[ELEVATION]);
                telemetry.addData("Sticker pitch", "%.1f°", py[STICKER_PITCH]);
                telemetry.addData("Roll", "%.1f°  %s", py[ROLL],
                        Math.abs(py[ROLL]) < 90 ? "right-side up" : "upside down");
            } else {
                telemetry.addLine("No cluster");
            }
             */

            // telemetry.addData("Inner Intake vel: ",innerIntakeVel);
            // telemetry.addData("Outer Intake vel: ",outerIntakeVel);
            telemetry.addData("turret spinning: ",activeLauncher);
            telemetry.addData("turret vel: ",launcherVel);
            // telemetry.addData("ball launch speed: ",launchSpeed);
            telemetry.addData("turret target pos: ",turretTicksToRotate);
            telemetry.addData("turret target angle: ",Math.toDegrees(angleToTarget));

            telemetry.addData("robot x: ",odometryPos.x);
            telemetry.addData("robot y: ",odometryPos.y);
            telemetry.addData("robot x dist: ",xDistToTarget);
            telemetry.addData("robot y dist: ",yDistToTarget);
            telemetry.addData("robot h: ",odometryPos.h);
            telemetry.addData("atan math: ",Math.atan2(xDistToTarget, yDistToTarget));
            telemetry.addData("manual turret controls: ",manualTurret);
            telemetry.addData("low power mode: ",lowPower);

            telemetry.addData("frontLeft power draw: ", frontLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rearLeft power draw: ", rearLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("frontRight power draw: ", frontRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rearRight power draw: ", rearRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("outerIntake power draw: ", outerIntake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("innerIntake power draw: ", innerIntake.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("turretSpin power draw: ", turretSpin.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("turretShoot power draw: ", turretShoot.getCurrent(CurrentUnit.AMPS));

            telemetry.update();
        }

        //limelight.stop();
    }
}