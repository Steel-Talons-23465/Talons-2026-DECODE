package org.firstinspires.ftc.teamcode.midas.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
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
    private PathChain launchToIntake1, intake1ToLaunch, launchtoIntake2, intake2ToLaunch;
    private Limelight3A limelight;
    private LLResult result;
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
        f.setStartingPose(poses.StartPose);
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

    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {

    }
    @Override
    public void stop(){

    }
    public int readTag(){

    }


}
