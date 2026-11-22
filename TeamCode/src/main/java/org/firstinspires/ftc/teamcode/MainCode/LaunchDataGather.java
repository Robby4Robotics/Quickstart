package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "LaunchDataGather" +
        "", group = "Test")

public class LaunchDataGather extends LinearOpMode {

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
    private double lowPower;
    private double outerIntakeVel;
    private double innerIntakeVel;
    private int activeIntake; // 0 = off, 1 = in, 2 = out
    private double prevBotHeading;
    private double launcherVel;
    private double outerIntakeTicks;
    private double innerIntakeTicks;

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

        // Odometry
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");

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

         odometry.setLinearUnit(DistanceUnit.METER);
         odometry.setAngularUnit(AngleUnit.DEGREES);
         // all of these below are temp values that need to be changed
         odometry.setOffset(new SparkFunOTOS.Pose2D(0.1885,0.035,180));
         odometry.setLinearScalar(1.463);
         odometry.setAngularScalar(1.0);
         odometry.resetTracking();







        // Change these if your Control Hub is mounted differently
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        lowPower = 1;
        outerIntakeVel = 0.0;
        innerIntakeVel = 0.0;
        outerIntakeTicks = 0.0;
        innerIntakeTicks = 0.0;
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
            double y = gamepad1.left_stick_y * lowPower;
            double x = -gamepad1.left_stick_x * lowPower;
            double rx = -gamepad1.right_stick_x * lowPower;

            // Get robot heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            SparkFunOTOS.Pose2D odometryPos = odometry.getPosition();

            /*
            // LIMELIGHT
            LLResult result = limelight.getLatestResult();
            double[] py = (result != null) ? result.getPythonOutput() : null;
             */
            double[] py = {};

            if (py != null && py[0] != -1) {
                if (py[0] == 30 || py[0] == 42) {
                    odometryPos = new SparkFunOTOS.Pose2D(1.5575 - py[2] * Math.cos(py[3]), 1.50625 - py[2] * Math.sin(py[3]), odometryPos.h);
                }
                else odometryPos = new SparkFunOTOS.Pose2D(2.1025 - py[2] * Math.cos(py[3]), 1.50625 - py[2] * Math.sin(py[3]), odometryPos.h);
                odometry.setPosition(odometryPos);
            }

            double xDistToTarget;
            if (odometryPos.x < 1.83) xDistToTarget = 1.5575 - odometryPos.x;
            else xDistToTarget = 2.1025 - odometryPos.x;

            double yDistToTarget = 1.50625 - odometryPos.y;

            // saving new target imu pos if rotated
            boolean hasRotated;
            if (Math.abs(rx) > 0.05) prevBotHeading = botHeading;

            // intake
            /*
            if (gamepad2.leftBumperWasPressed()) {
                outerIntakeTicks += 0.05;
            } else if (gamepad2.leftTriggerWasPressed()) {
                outerIntakeTicks -= 0.05;
            }
            if (Math.abs(outerIntakeTicks) > VELCONST) {
                outerIntakeTicks = Math.copySign(VELCONST,outerIntakeTicks);
            }
            if (gamepad2.leftBumperWasPressed()) {
                innerIntakeTicks += 0.05;
            } else if (gamepad2.leftTriggerWasPressed()) {
                innerIntakeTicks -= 0.05;
            }
            if (Math.abs(innerIntakeTicks) > VELCONST) {
                innerIntakeTicks = Math.copySign(VELCONST,innerIntakeTicks);
            }

            if (gamepad1.aWasPressed()) {
                if (activeIntake == 1) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = innerIntakeTicks;
                    activeIntake = 1;
                }

            } else if (gamepad1.bWasPressed()) {
                if (activeIntake == 2) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = -innerIntakeTicks;
                    activeIntake = 2;
                }

            }

            if (gamepad2.b) {
                innerIntakeVel = outerIntakeTicks;
            }
            else if (gamepad2.a) {
                innerIntakeVel = -outerIntakeTicks;
            }
             */
            if (gamepad1.aWasPressed()) {
                if (activeIntake == 1) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = VELCONST * 0.5;
                    activeIntake = 1;
                }

            } else if (gamepad1.bWasPressed()) {
                if (activeIntake == 2) {
                    outerIntakeVel = 0;
                    activeIntake = 0;
                }
                else {
                    outerIntakeVel = -VELCONST * 0.5;
                    activeIntake = 2;
                }

            }

            if (gamepad2.b) {
                innerIntakeVel = VELCONST * 0.6;
            }
            else if (gamepad2.a) {
                innerIntakeVel = -VELCONST * 0.6;
            }

            if (gamepad1.rightBumperWasPressed()){
                if (lowPower == 1) {
                    lowPower = 0.25;
                } else {
                    lowPower = 1;
                }
            }

            if (gamepad1.leftBumperWasPressed()){
                imu.resetYaw();
                odometry.setPosition(new SparkFunOTOS.Pose2D(1.0266,0.02,180));
            }

            // launcher
            if (gamepad2.dpadLeftWasPressed()) {
                launcherVel = 0;
            }
            else if (gamepad2.dpadDownWasPressed()) {
                launcherVel -= 0.05 * VELCONST;
            }
            else if (gamepad2.dpadUpWasPressed()) {
                launcherVel += 0.01 * VELCONST;
            }
            else if (gamepad2.dpadRightWasPressed()) {
                launcherVel += 0.1 * VELCONST;
            }
            if (launcherVel > VELCONST) launcherVel = VELCONST;
            if (launcherVel < -VELCONST) launcherVel = -VELCONST;

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

            // Send velocity to motors
            frontLeft.setVelocity(frontLeftVel);
            frontRight.setVelocity(frontRightVel);
            rearLeft.setVelocity(rearLeftVel);
            rearRight.setVelocity(rearRightVel);

            outerIntake.setVelocity(outerIntakeVel);
            innerIntake.setVelocity(innerIntakeVel);

            turretShoot.setVelocity(launcherVel);

            // Telemetry
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
            telemetry.addData("turret vel: ",launcherVel);
            // telemetry.addData("ball launch speed: ",launchSpeed);

            telemetry.addData("robot x: ",odometryPos.x);
            telemetry.addData("robot y: ",odometryPos.y);
            telemetry.addData("robot x dist: ",xDistToTarget);
            telemetry.addData("robot y dist: ",yDistToTarget);
            telemetry.addData("robot h: ",odometryPos.h);
            telemetry.addData("low power mode: ",lowPower);

            telemetry.addData("Inner intake vel: ",innerIntakeTicks);
            telemetry.addData("Outer intake vel: ",outerIntakeTicks);

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