package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class MidasConstants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.5)
            .forwardZeroPowerAcceleration(-30.8969154803)
            .lateralZeroPowerAcceleration(-66.4346399703)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)

            .translationalPIDFCoefficients(new PIDFCoefficients(1, .001, .15, .04))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, .095, .04))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(.05, 0, .00065, 0, .020))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients( 0, 0, 0, .03)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(.75, 0, 0, .03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(.15,0,0.01,0.015))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(65.4189987543)
            .yVelocity(51.7807841143);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.30709)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .yawScalar(1.0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            .95,
            75,
            1,
            1

    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}