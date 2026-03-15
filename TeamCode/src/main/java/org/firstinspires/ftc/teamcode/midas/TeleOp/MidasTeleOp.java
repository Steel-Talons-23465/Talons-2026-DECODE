package org.firstinspires.ftc.teamcode.midas.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.Midas;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.MidasPoseConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;

@TeleOp
public class MidasTeleOp extends LinearOpMode {
    double speedMultiplier =1;
    double tX;
    Follower f;
    MidasPoseConstants poses = new MidasPoseConstants();
    Limelight3A limelight;

    Midas midas = new Midas();

    boolean odo, tag, off;
    boolean isOdo, isTag , aprilTagDetected;

    private double theta, diffX, diffY;
    private Vector vel = new Vector(new Pose(0,0,0));
    private int odoTicks, tagTicks;
    private LLResult result;

    public boolean launching;
    public boolean manualAngle = false, manualVel = false;
    public double launchVelocity;
    public double angleHood = .25;


    double hoodAngle = .25;

    public void runOpMode() throws InterruptedException{
        midas.initialize(hardwareMap);
        f = MidasConstants.createFollower(hardwareMap);
        waitForStart();
        f.setStartingPose(SharedData.toTeleopPose == null ? new Pose(0,0,0): SharedData.toTeleopPose);

        limelight = hardwareMap.get(Limelight3A.class , "limelight");
        limelight.start();
        limelight.pipelineSwitch(SharedData.side == Side.RED ? 1 : 2);
        f.startTeleOpDrive(true);
        while (opModeIsActive()){
            f.update();
            updateMovement();


            if (gamepad1.b && !odo){
                isOdo = true;
            }
            else if (gamepad1.y && !tag){
                isTag=true;
            }
            else if (gamepad1.x && !off)
            {
                isOdo = false;
                isTag=false;
                telemetry.addData("currently" , "nothing");
            }
            if (gamepad1.right_bumper){
                midas.startIntake(true);
            }else if (gamepad1.left_bumper){
                midas.startIntake(false);
            }
            else midas.stopIntake();
            trackingWithOdo(isOdo);
            trackingWithTag(isTag);

            hoodAngle = manualAngle ? angleHood : regressedAngle();
            if(hoodAngle > .65)
                hoodAngle = .65;
            if (hoodAngle < .25)
                hoodAngle = .25;
            midas.setHoodAngle(hoodAngle);

            launchVelocity = manualVel ? launchVelocity : regressedPower();
            if(launching)
                midas.setLaunchVelocity(launchVelocity);
            else
                midas.setLaunchVelocity(0);

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

    private void trackingWithOdo(boolean usingOdo) {
        if (usingOdo){

            diffX = Math.abs(0 - f.getPose().getX());
            diffY = Math.abs(144 - f.getPose().getY());
            telemetry.addData("theta" , ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            theta = ((midas.getTurretPos() < 1000 ? -f.getPose().getHeading()*180/Math.PI : -f.getPose().getHeading()*180/Math.PI+360) + 90 + ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            telemetry.addData("angle", theta);
            odoTicks =  (int) ((theta)*5.771);

            if (odoTicks >= 2400){
                odoTicks = odoTicks-2077;
            }
            else if (midas.getTurretTargetPos() <= -775){
                odoTicks = odoTicks+2077;
            }
            midas.setTurretTargetPos(odoTicks);
            telemetry.addData("currently" , "odo");

        }
    }
    private void trackingWithTag(boolean usingTag) {
        if (usingTag) {
            senseTag();
            if (Math.abs(tX) > 1) {
                if(Math.abs(midas.getTurretPos() - midas.getTurretTargetPos()) < 120)
                    tagTicks = midas.getTurretPos() - (int) (5.771 * tX);
                if (tagTicks >= 2400){
                    tagTicks = tagTicks-2077;
                }
                else if (midas.getTurretTargetPos() <= -775){
                    tagTicks = tagTicks+2077;

                }
                midas.setTurretTargetPos(tagTicks);
            } else {
                if(aprilTagDetected)
                    midas.setTurretTargetPos(tagTicks);
                telemetry.addData("currently", "tag");

            }
        }
    }
    public double regressedAngle(){
        return (0.00210679 * calculateDist() ) + (0.000785782 * midas.getLaunchVelocity())+ (-0.00000240321 * calculateDist() * midas.getLaunchVelocity()) - 0.578298;
    }
    public double regressedPower(){
        return (0.0287961 * calculateDist() * calculateDist() ) + (-0.847604 * calculateDist())  +1199.26456;
        //return (4.98089 * calculateDist())+918.9466;
        //return (4.89581 * calculateDist())+938.18835;
    }
    public double calculateDist(){
        return Math.sqrt(Math.pow(Math.abs(f.getPose().getX() - 0) , 2) + Math.pow(Math.abs(f.getPose().getY() - 144) , 2));
    }




    public void senseTag () {
        result = limelight.getLatestResult();
        try {
            tX = result.getFiducialResults().get(0).getTargetXDegrees();
            telemetry.addData("Reading Apriltag", result.getFiducialResults().get(0).getFiducialId());
            telemetry.addData("TagX", tX);
            aprilTagDetected = true;

        } catch (Exception e) {
            tX = 0;
            aprilTagDetected = false;
            telemetry.addData("No AprilTag Detected", "-1");
        }

    }




}
