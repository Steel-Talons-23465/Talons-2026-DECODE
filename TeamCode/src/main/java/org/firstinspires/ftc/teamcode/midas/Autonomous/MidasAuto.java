package org.firstinspires.ftc.teamcode.midas.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.Midas;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.MidasCalibration;
import org.firstinspires.ftc.teamcode.midas.MidasGeneral.MidasPoseConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;


public class MidasAuto extends OpMode {

    private Midas midas = new Midas();
    private Follower f;
    private int pathState, index;
    private Timer pathTimer, opModeTimer, launchTimer;
    private MidasPoseConstants poses = new MidasPoseConstants();
    private Path start, end;
    private PathChain launchToAlign1, align1ToIntake1, intake1ToLaunch, launchToAlign2, align2ToIntake2,  intake2ToLaunch;
    private Limelight3A limelight;
    private LLResult result;
    private boolean launching, manualAngle = false;
    private double angleHood, hoodAngle;
    @Override
    public void init() {
        midas.initialize(hardwareMap);
        SharedData.reset();
        pathTimer = new Timer();
        opModeTimer = new Timer();
        launchTimer = new Timer();
        opModeTimer.resetTimer();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        f = MidasConstants.createFollower(hardwareMap);
        f.setStartingPose(poses.START_POSE);
        buildPaths();
        //midas resets whatever needs resetting
        limelight.start();
        limelight.pipelineSwitch(0);
    }
    @Override
    public void init_loop(){
        int ID = figureID();
        if (ID == 21) index = 0;
        else if (ID == 22) index = 1;
        else if (ID == 23) index = 2;

        SharedData.greenIndex = index;
        telemetry.addData("ID" , ID);
        telemetry.addData("Green Index", index );
        telemetry.addData("Side", SharedData.side);
        telemetry.addData("Start", SharedData.startFar ? "far" : "close");
        telemetry.addData("Shoot", SharedData.shootFar ? "far" : "close");
        telemetry.update();

    }
    @Override
    public void start(){
        sendPose();
        opModeTimer.resetTimer();
        limelight.pipelineSwitch(SharedData.side == Side.RED ? 1 : 2);
        setPathState(0);


    }

    public void setPathState(int pathState){
        this.pathState = pathState;
        pathTimer.resetTimer();
    }


    public void autoPathUpdates(){
        sendPose();
        switch (this.pathState){

            case 0:
                f.followPath(start);
                setPathState(1);
                break;
            case 1:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    f.followPath(launchToAlign1, true);
                    setPathState(2);
                    launching = false;
                }
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() >1.5){
                    launching = true;
                }
                break;

            case 2:
                if (!f.isBusy()){
                   f.followPath(align1ToIntake1,true);
                   f.setMaxPower(.25);
                   setPathState(3);
                }
                break;

            case 3:
                if (!f.isBusy() || SharedData.isFull()){
                    f.followPath(intake1ToLaunch);
                    f.setMaxPower(1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!f.isBusy() && SharedData.isEmpty()){
                    f.followPath(launchToAlign2);
                    setPathState(5);
                    launching = false;
                }
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() >1.5){
                    launching = true;
                }
                break;
            case 5:
                if (!f.isBusy() ){
                    f.followPath(align2ToIntake2);
                    f.setMaxPower(.25);
                    setPathState(6);
                }
                break;
            case 6:
                if (!f.isBusy() || SharedData.isFull()){
                    f.followPath(intake2ToLaunch);
                    f.setMaxPower(1);
                    setPathState(7);
                }
                break;
            case 7:
                if (!f.isBusy() && SharedData.isEmpty()){
                    f.followPath(end);
                    setPathState(8);
                    launching = false;
                }
                else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5){
                    launching = true;
                }
                break;
            case 8:
                if (!f.isBusy()){
                    setPathState(-1);
                    launching = false;
                }

        }







    }
    public void buildPaths(){
        start = new Path(new BezierLine(poses.START_POSE , poses.LAUNCH_POSE));
        start.setLinearHeadingInterpolation(poses.START_POSE.getHeading() , poses.LAUNCH_POSE.getHeading());


        end = new Path(new BezierLine(poses.LAUNCH_POSE, poses.END_POSE));
        end.setLinearHeadingInterpolation(poses.LAUNCH_POSE.getHeading(), poses.END_POSE.getHeading());
    }


    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();
        hoodAngle = manualAngle ? angleHood : regressedAngle();
        if(hoodAngle > .65)
            hoodAngle = .65;
        if (hoodAngle < .25)
            hoodAngle = .25;
        midas.setHoodAngle(hoodAngle);
        if (!launching){
            if (pathState == 1 || pathState == 4 || pathState == 7 ){
                midas.setLaunchVelocity(regressedPower());
            }
            else midas.setLaunchVelocity(0);
            if (launchTimer.getElapsedTimeSeconds() > .1){

            }
        }

    }
    @Override
    public void stop(){

    }
    public int figureID(){
        result = limelight.getLatestResult();

        try {
            telemetry.addData("ID" , result.getFiducialResults().get(0).getFiducialId());
            return result.getFiducialResults().get(0).getFiducialId();

        }
        catch (Exception e){
            telemetry.addData("ID" ,"not found");
            return -1;
        }

    }
    public void sendPose(){
        SharedData.toTeleopPose = f.getPose();
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

}
