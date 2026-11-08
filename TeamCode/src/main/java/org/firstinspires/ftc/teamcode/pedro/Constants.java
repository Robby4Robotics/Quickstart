package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontleft");
                c.backLeftName.set("rearleft");
                c.frontRightName.set("frontright");
                c.backRightName.set("rearright");
                c.frontLeftDirection.set(DcMotorEx.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorEx.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorEx.Direction.REVERSE);
                c.backRightDirection.set(DcMotorEx.Direction.REVERSE);
            }
    );
    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("odometry");
        c.linearScalar.set(9.467001327014218);
        c.angularScalar.set(1.0266259166023104);
        c.offset.set(new Pose(0.15619232898622049, -0.2463032880167323));
        c.linearUnit.set(DistanceUnit.INCH);
    });


}