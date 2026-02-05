package org.firstinspires.ftc.teamcode.TeleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.General.AutoPathStates;
import org.firstinspires.ftc.teamcode.General.ColorSensed;
import org.firstinspires.ftc.teamcode.General.PoseConstants;


import com.pedropathing.paths.PathChain;


import org.firstinspires.ftc.teamcode.General.Robot;
import org.firstinspires.ftc.teamcode.General.SharedData;
import org.firstinspires.ftc.teamcode.General.Side;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "New TeleOp")
public class NewTeleOp extends LinearOpMode{
    private Robot hornet = new Robot();
    private Follower f;
    private PoseConstants poses = new PoseConstants();
    boolean robotCentric;
    double speedMultiplier;
    boolean slowMode;
    boolean launching;
    private Timer launchTimer;
    private int inMotif;
    AutoPathStates setPose = AutoPathStates.TELEOP;

    boolean dpadLeft;
    boolean dpadLeft2;

    public void runOpMode(){
        hornet.initialize(hardwareMap);
        f = Constants.createFollower(hardwareMap);
        f.setStartingPose(SharedData.toTeleopPose == null ? poses.START_POSE : SharedData.toTeleopPose);
        f.update();
        launchTimer = new Timer();
        hornet.resetHammer();

        waitForStart();

        f.startTeleOpDrive(true);

        while(opModeIsActive()) {
            f.update();

            if(setPose == AutoPathStates.TELEOP) {
                f.setTeleOpDrive(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier,
                        robotCentric,
                        (SharedData.side == Side.RED || robotCentric) ? 0 : Math.toRadians(180)
                );
            }

            if (setPose == AutoPathStates.TELEOP){
                //go to close launch main
                if (gamepad1.a){
                    f.followPath(goToClose(false));
                    setPose = AutoPathStates.CLOSE;
                }
                //go to close launch alt
                else if (gamepad2.a  && !gamepad2.right_bumper && !gamepad2.left_bumper){
                    f.followPath(goToClose(true));
                    setPose = AutoPathStates.CLOSE_ALT;
                }
                //go to park
                else if (gamepad1.x){
                    f.followPath(goToPark());
                    setPose = AutoPathStates.PARK;
                }
                //go to far launch main
                else if (gamepad1.b){
                    f.followPath(goToFar(false));
                    setPose = AutoPathStates.FAR;
                }
                //go to far launch alt
                else if (gamepad2.b && !gamepad2.right_bumper && !gamepad2.left_bumper ){
                    f.followPath(goToFar(true));
                    setPose = AutoPathStates.FAR_ALT;
                }
                //go to gate
                else if ((gamepad1.y ||(gamepad2.y && !gamepad2.right_bumper && !gamepad2.left_bumper))) {
                    f.followPath(goToGate());
                    setPose = AutoPathStates.GATE;
                }
            }
            //exits automated
            else if((!gamepad1.a && setPose == AutoPathStates.CLOSE)
                    || (!gamepad2.a && setPose == AutoPathStates.CLOSE_ALT)
                    || ((!gamepad1.y && !gamepad2.y) && setPose == AutoPathStates.GATE)
                    || (!gamepad1.x && setPose == AutoPathStates.PARK)
                    || (!gamepad1.b && setPose == AutoPathStates.FAR)
                    || (!gamepad2.b && setPose == AutoPathStates.FAR_ALT)){
                f.startTeleopDrive(true);
                setPose = AutoPathStates.TELEOP;
            }


            telemetry.addData("in" , setPose);
            telemetry.update();

            if(gamepad1.left_trigger >= .2)
                speedMultiplier = .2;
            else
                speedMultiplier = 1;


            if(SharedData.isFull()&&!launching && hornet.atSortTarget())
                hornet.startIntake(false,.2);
            else if(gamepad1.right_bumper)
                hornet.startIntake(true,1);
            else if(gamepad1.left_bumper)
                hornet.startIntake(false,1);
            else
                hornet.stopIntake();

            hornet.updateLED();

            if(dpadLeft != gamepad1.dpad_left && gamepad1.dpad_left)
                robotCentric = !robotCentric;
            dpadLeft = gamepad1.dpad_left;
        }
    }

    public void autoMode() {

        if((gamepad2.right_trigger > .2 && gamepad2.left_trigger > .2) || gamepad1.dpad_down)
        {
            launching = false;
            hornet.resetLaunch();
        }


        if(!launching)
            hornet.setStoragePos(SharedData.storage[0] == ColorSensed.NO_COLOR ? 0 : (SharedData.storage[1] == ColorSensed.NO_COLOR ? 1 : 2) , !SharedData.isFull());

        if(gamepad2.right_bumper && gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.clearSlot(0);
            else if(gamepad2.b)
                SharedData.clearSlot(1);
            else if(gamepad2.y)
                SharedData.clearSlot(2);
        }
        else if(gamepad2.right_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.GREEN;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.GREEN;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.GREEN;
        }
        else if(gamepad2.left_bumper) {
            if(gamepad2.a)
                SharedData.storage[0] = ColorSensed.PURPLE;
            else if(gamepad2.b)
                SharedData.storage[1] = ColorSensed.PURPLE;
            else if(gamepad2.y)
                SharedData.storage[2] = ColorSensed.PURPLE;
        }


        if(hornet.buttonPressed() && hornet.atSortTargetLenient() && SharedData.storage[hornet.getSlotGoal()] == ColorSensed.NO_COLOR){
            SharedData.storage[hornet.getSlotGoal()] = hornet.detectColor();
        }


        if(gamepad2.dpad_up && SharedData.getGreenIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getGreenIndex(), false);
            launching = true;
        }
        if(gamepad2.dpad_down && SharedData.getPurpleIndex() != -1 && !launching)
        {
            hornet.setStoragePos(SharedData.getPurpleIndex(), false);
            launching = true;
        }
        if(gamepad2.dpad_right && !SharedData.isEmpty() && !launching)
        {
            hornet.setStoragePos(inMotif == SharedData.greenIndex ? (SharedData.getGreenIndex() == -1 ? SharedData.getPurpleIndex() : SharedData.getGreenIndex()) : (SharedData.getPurpleIndex() == -1 ? SharedData.getGreenIndex() : SharedData.getPurpleIndex()), false);
            launching = true;
        }
        if(dpadLeft2 != gamepad2.dpad_left && gamepad2.dpad_left)
            inMotif  = inMotif == 2 ? 0 : inMotif + 1;
        dpadLeft2 = gamepad2.dpad_left;


        //If launching -> speed up launchMotors
        if(launching){
            if(SharedData.side == Side.RED ? (f.getPose().getX() < 72) : (f.getPose().getX() > 72)) {
                hornet.startLaunchMotorsAlt(f.getPose().getY() < 48);
            }
            else {
                hornet.startLaunchMotors(f.getPose().getY() < 48);
            }
            hornet.startIntake(true, .1);
        } else{hornet.stopLaunchMotors();}

        //if ready to launch -> then launch
        if(launching && hornet.atSortTarget() && hornet.atTargetVelocity() && !hornet.hammerAtLaunch() && !hornet.isLaunched()){
            hornet.launch();
            launchTimer.resetTimer();
        }

        //if flap has had time to move...
        //and flap is at launch position -> move flap back and clear storage slot
        //and flap is at not launch position and it says its launching -> say its not launching

        if(hornet.hammerAtLaunch() && launching && !hornet.atTargetVelocity() && launchTimer.getElapsedTimeSeconds() > .25){
            launchTimer.resetTimer();
            hornet.resetHammer();
            SharedData.clearSlot(hornet.getSlotGoal());
        }

        if(launching && hornet.isLaunched() && !hornet.hammerAtLaunch() && launchTimer.getElapsedTimeSeconds() > .25){
            launching = false;
            inMotif  = inMotif == 2 ? 0 : inMotif + 1;
            hornet.resetLaunch();
        }



    }



    public PathChain goToClose(boolean alt){
        if (alt){ return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.closeAlt))
                .setConstantHeadingInterpolation(poses.closeAlt.getHeading())
                .build();
        }
        else return f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.closeLaunch))
                .setConstantHeadingInterpolation(poses.closeLaunch.getHeading())
                .build();
    }
    public PathChain goToFar(boolean alt){
        if (alt){ return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.farAlt))
                .setConstantHeadingInterpolation(poses.farAlt.getHeading())
                .build();
        }
        else return f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.farLaunch))
                .setLinearHeadingInterpolation(f.getHeading() , poses.farLaunch.getHeading())
                .build();
    }


    public PathChain goToPark()
    {
        PathChain toPark = f.pathBuilder()
                .addPath(new BezierLine(f.getPose(), poses.parkPose))
                .setConstantHeadingInterpolation(poses.parkPose.getHeading())
                .build();
        return toPark;
    }
//    public PathChain goToClose()
//    {
//        PathChain toLaunchClose = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), poses.closeLaunch))
//                .setLinearHeadingInterpolation(f.getPose().getHeading(), poses.closeLaunch.getHeading())
//                .build();
//        return toLaunchClose;
//    }

    public PathChain goToGate(){
        return f.pathBuilder()
                .addPath(new BezierLine(f.getPose() , poses.gatePose))
                .setConstantHeadingInterpolation(poses.gatePose.getHeading())
                .build();

    }

}
