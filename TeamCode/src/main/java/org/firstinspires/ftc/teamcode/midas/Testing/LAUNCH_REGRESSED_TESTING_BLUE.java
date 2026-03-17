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
public class LAUNCH_REGRESSED_TESTING_BLUE extends LinearOpMode {

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
    public static boolean isOdo, isTag, aprilTagDetected;

    public static boolean launching;
    public static double launchVelocity;
    public static double angleHood = .25;
    public static double poseX, poseY, poseH, headingInAngle;
    private Pose robotPose= new Pose(poseX,poseY,poseH);
    //private final Pose goal = MidasPoseConstants.goal;
public static boolean manualAngle, manualVel, manualTurret;
    double hoodAngle = .25;



    public void runOpMode() throws InterruptedException{
        poseX = 0;
        poseY = 0;
        poseH = 0;
        f = MidasConstants.createFollower(hardwareMap);

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        angleLeft = hardwareMap.get(Servo.class, "angleLeft");
        angleRight = hardwareMap.get(Servo.class, "angleRight");

        turret = hardwareMap.get(DcMotorEx.class , "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight = hardwareMap.get(Limelight3A.class , "limelight");

        limelight.start();
        limelight.pipelineSwitch(2);
        f.setStartingPose(midasPoses.START_POSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        f.startTeleOpDrive(true);

        while (opModeIsActive()){
            robotPose = new Pose(poseX , poseY , poseH);
            f.update();
            currentPos = turret.getCurrentPosition();
//            if (gamepad1.a && !heading){
//                isOdo = false;
//                isTag=false;
//            }
//            else if ((gamepad1.b || dashOdo) && !odo){
//                isOdo = true;
//            }
//            else if ((gamepad1.y || dashTag) && !tag){
//                isTag=true;
//            } else if ((gamepad1.x || dashOff) && !off)
//            {
//                isOdo = false;
//                isTag=false;
//                telemetry.addData("currently" , "nothing");
//            }
            poseH = Math.toRadians(headingInAngle);
            trackingWithOdo(isOdo);
            trackingWithTag(isTag);

            turret.setPower(manualTurret ? 0: .5);

            odo = gamepad1.b;
            heading = gamepad1.a;
            tag = gamepad1.y;
            off = gamepad1.x;


            hoodAngle = manualAngle ? angleHood : regressedAngle();
            if(hoodAngle > .65)
                hoodAngle = .65;
            if (hoodAngle < .25)
                hoodAngle = .25;
            setHoodAngle(hoodAngle);

            launchVelocity = manualVel ? launchVelocity : regressedPower();
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
            packet.put(" Distance", calculateDist());
            packet.put("regressed power" , regressedPower());
            packet.put("regressed angle" , regressedAngle());

            dashboard.sendTelemetryPacket(packet);


            telemetry.addData("odo" ,odo );
            telemetry.addData("tag", tag);
            telemetry.addData("heading ", heading);
            telemetry.addData("off ", off);
            telemetry.addData("regressed power" , regressedPower());
            telemetry.addData("regressed angle" , regressedAngle());


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

            diffX = Math.abs(0 - poseX);
            diffY = Math.abs(144 - poseY);
            telemetry.addData("theta" , ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            theta = ((turret.getCurrentPosition() < 1000 ? -poseH*180/Math.PI : -poseH*180/Math.PI+360) + 90 + ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
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
                if (Math.abs(currentPos - turret.getTargetPosition()) < 120)
                    tagTicks = currentPos - (int) (5.771 * tX);
                if (tagTicks >= 2400) {
                    tagTicks = tagTicks - 2077;
                } else if (turret.getTargetPosition() <= -775) {
                    tagTicks = tagTicks + 2077;

                }
                turret.setTargetPosition(tagTicks);
            } else {
                if (aprilTagDetected)
                    turret.setTargetPosition(tagTicks);
                telemetry.addData("currently", "tag");

            }
        }
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

    public double calculateDist(){
        return Math.sqrt(Math.pow(Math.abs(robotPose.getX() - 0) , 2) + Math.pow(Math.abs(robotPose.getY() - 144) , 2));
    }
    public double regressedAngle(){
        return (0.00210679 * calculateDist() ) + (0.000785782 * launch.getVelocity())+ (-0.00000240321 * calculateDist() * launch.getVelocity()) - 0.578298;
    }
    public double regressedPower(){
        return (0.0287961 * calculateDist() * calculateDist() ) + (-0.847604 * calculateDist())  +1199.26456;
        //return (4.98089 * calculateDist())+918.9466;
        //return (4.89581 * calculateDist())+938.18835;
    }
}

