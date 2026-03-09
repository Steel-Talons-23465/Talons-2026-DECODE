package org.firstinspires.ftc.teamcode.midas.Testing;

import static org.firstinspires.ftc.teamcode.hornet.General.Side.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hornet.General.PoseConstants;
import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.MidasPoseConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;
@Config
@TeleOp
public class TURRET_POWER_ANGLE_TESTING extends LinearOpMode {

    private Follower f;
    private PoseConstants poses = new PoseConstants();
    private MidasPoseConstants midasPoses = new MidasPoseConstants();
    DcMotorEx launch = null;
    DcMotorEx turret = null;
    Servo angleLeft = null;
    Servo angleRight = null;
    private double speedMultiplier, tX, tY;
    private double theta, diffX, diffY;
    private Vector vel = new Vector(new Pose(0,0,0));
    private int currentPos;
    private int odoTicks, tagTicks;
    private Limelight3A limelight = null;
    private LLResult result;
    boolean odo, tag, heading, off;
    boolean isOdo, isTag, aprilTagDetected;

    public static boolean launching;
    public static double launchVelocity;
    public static double angleHood = .25;

    double hoodAngle = .25;



    public void runOpMode() throws InterruptedException{

        f = MidasConstants.createFollower(hardwareMap);

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        angleLeft = hardwareMap.get(Servo.class, "angleLeft");
        angleRight = hardwareMap.get(Servo.class, "angleRight");

        turret = hardwareMap.get(DcMotorEx.class , "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setVelocity(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        limelight.start();
        limelight.pipelineSwitch(SharedData.side == Side.RED ? 1 : 2);
        f.setStartingPose(midasPoses.StartPose);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        f.startTeleOpDrive(true);

        while (opModeIsActive()){
            f.update();
            currentPos = turret.getCurrentPosition();
            vel = f.getVelocity();
            updateMovement();
            if (gamepad1.a && !heading){
                isOdo = false;
                isTag=false;
            }
            else if (gamepad1.b && !odo){
                isOdo = true;
            }
            else if (gamepad1.y && !tag){
                isTag=true;
            } else if (gamepad1.x && !off)
            {
                isOdo = false;
                isTag=false;
                telemetry.addData("currently" , "nothing");
            }

            trackingWithOdo(isOdo);
            trackingWithTag(isTag);



            odo = gamepad1.b;
            heading = gamepad1.a;
            tag = gamepad1.y;
            off = gamepad1.x;


            hoodAngle = angleHood;
            if(hoodAngle > .65)
                hoodAngle = .65;
            if (hoodAngle < .25)
                hoodAngle = .25;
            setHoodAngle(hoodAngle);

            if(launching)
                setLaunchVelocity(launchVelocity);
            else
                setLaunchVelocity(0);


            double odoDistance = Math.sqrt(Math.pow(midasPoses.goal.getX() - f.getPose().getX(),2) + Math.pow(midasPoses.goal.getY() - f.getPose().getY(),2));


            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Velocity", launchVelocity);
            packet.put("Actual Velocity", launch.getVelocity());
            packet.put("Hood Angle", hoodAngle);
            packet.put("Launching", launching);
            packet.put("Odo Distance", odoDistance);
            dashboard.sendTelemetryPacket(packet);


            telemetry.addData("odo" ,odo );
            telemetry.addData("tag", tag);
            telemetry.addData("heading ", heading);
            telemetry.addData("off ", off);

            telemetry.addData("currentPos" , turret.getCurrentPosition());

            telemetry.addData("fx", f.getPose().getX());
            telemetry.addData("fy", f.getPose().getY());
            telemetry.addData("fh", f.getPose().getHeading());
            telemetry.update();

//2597 = 1.25 rotation direction (ccw);
//1558 = .75 rotation direction cw
//
        }

    }

    private void trackingWithOdo(boolean usingOdo) {
        if (usingOdo){

            diffX = Math.abs(midasPoses.goal.getX() - f.getPose().getX());
            diffY = Math.abs(midasPoses.goal.getY() - f.getPose().getY());
            telemetry.addData("theta" , ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            theta = ((turret.getCurrentPosition() < 1000 ? -f.getPose().getHeading()*180/Math.PI : -f.getPose().getHeading()*180/Math.PI+360) + 90 + ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            telemetry.addData("angle", theta);
            odoTicks =  (int) ((theta)*5.771);

            if (odoTicks >= 2400){
                odoTicks = odoTicks-2077;
            }
            else if (turret.getTargetPosition() <= -775){
                odoTicks = odoTicks+2077;
            }
            turret.setTargetPosition(odoTicks);
            telemetry.addData("currently" , "odo");

        }
    }

    private void trackingWithTag(boolean usingTag) {
        if (usingTag) {
            senseTag();
            if (Math.abs(tX) > 1) {
                if(Math.abs(currentPos - turret.getTargetPosition()) < 120)
                    tagTicks = currentPos - (int) (5.771 * tX);
                if (tagTicks >= 2400){
                    tagTicks = tagTicks-2077;
                }
                else if (turret.getTargetPosition() <= -775){
                    tagTicks = tagTicks+2077;

                }
                turret.setTargetPosition(tagTicks);
            } else {
                if(aprilTagDetected)
                    turret.setTargetPosition(tagTicks);
                telemetry.addData("currently", "tag");

            }
        }
    }

    private void updateMovement () {

        f.setTeleOpDrive(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                true,
                0);

        if (gamepad1.left_trigger >= .2)
            speedMultiplier = .2;
        else
            speedMultiplier = 1;
    }


    public void senseTag () {
        result = limelight.getLatestResult();
        try {
            tX = result.getFiducialResults().get(0).getTargetXDegrees();
            tY = result.getFiducialResults().get(0).getTargetYDegrees();
            telemetry.addData("Reading Apriltag", result.getFiducialResults().get(0).getFiducialId());
            telemetry.addData("TagX", tX);
            telemetry.addData("TagY", tY);
            aprilTagDetected = true;

        } catch (Exception e) {
            tX = 0;
            tY = 0;
            aprilTagDetected = false;
            telemetry.addData("No AprilTag Detected", "-1");
        }

    }

    public void setHoodAngle(double position) {
        angleLeft.setPosition(position); //0 -.6
        angleRight.setPosition(1 - position); //.25-1
    }

    public void setLaunchVelocity(double velocity){
        launch.setVelocity(velocity);
    }
}

