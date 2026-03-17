package org.firstinspires.ftc.teamcode.midas.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
    private int pathState, index, ID;
    private double tX;
    private int tagTicks;
    private boolean aprilTagDetected;
    private Timer pathTimer, opModeTimer, launchTimer;
    private MidasPoseConstants poses = new MidasPoseConstants();
    private Path start_to_launch, end;
    private PathChain launchToAlign1, align1ToIntake1, intake1ToLaunch, launchToIntake2, intake2ToLaunch, launchToIntake3, intake3ToLaunch, launchToIntake4;
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
    public void init_loop() {
        if (ID == 21) index = 0;
        else if (ID == 22) index = 1;
        else if (ID == 23) index = 2;

        SharedData.greenIndex = index;
        telemetry.addData("ID", ID);
        telemetry.addData("Green Index", index);
        telemetry.addData("Side", SharedData.side);
        telemetry.addData("Start", SharedData.startFar ? "far" : "close");
        telemetry.addData("Shoot", SharedData.shootFar ? "far" : "close");
        telemetry.update();

    }

    @Override
    public void start() {
        sendPose();
        opModeTimer.resetTimer();
        limelight.pipelineSwitch(SharedData.side == Side.RED ? 1 : 2);
        setPathState(0);


    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }


    public void autoPathUpdates() {
        sendPose();
        switch (this.pathState) {

            case 0:
                f.followPath(start_to_launch);
                setPathState(1);
                break;
            case 1:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    f.followPath(launchToAlign1, true);
                    setPathState(2);
                    launching = false;
                } else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    launching = true;
                }
                break;

            case 2:
                if (!f.isBusy()) {
                    f.followPath(align1ToIntake1, true);
                    f.setMaxPower(.25);
                    setPathState(3);
                }
                break;

            case 3:
                if (!f.isBusy() || SharedData.isFull()) {
                    f.followPath(intake1ToLaunch);
                    f.setMaxPower(1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    f.followPath(launchToIntake2);
                    setPathState(5);
                    launching = false;
                } else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    launching = true;
                }
                break;
            case 5:
                if (!f.isBusy()) {
                    f.followPath(intake2ToLaunch);
                    setPathState(6);
                }
                break;
            case 6:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    f.followPath(launchToIntake3);
                    setPathState(7);
                    launching = false;
                } else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    launching = true;
                }
                break;
            case 7:
                if (!f.isBusy()) {
                    f.followPath(intake3ToLaunch);
                    setPathState(8);
                }
            case 8:
                if (!f.isBusy() && SharedData.isEmpty()) {
                    f.followPath(launchToIntake4);
                    setPathState(9);
                    launching = false;
                } else if (!f.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    launching = true;
                }
            case 9:
                if (!f.isBusy() || SharedData.isFull()) {
                    f.followPath(end);
                    setPathState(10);
                }
            case 10:
                if (!f.isBusy()) {
                    setPathState(-1);
                    launching = false;
                }

        }


    }

    public void buildPaths() {
        start_to_launch = new Path(new BezierLine(poses.START_POSE, poses.LAUNCH_POSE));
        start_to_launch.setConstantHeadingInterpolation(poses.START_POSE.getHeading());

        launchToAlign1 = f.pathBuilder()
                .addPath(new BezierLine(poses.LAUNCH_POSE, poses.ALIGN1_POSE))
                .setConstantHeadingInterpolation(poses.ALIGN1_POSE.getHeading())
                .build();

        align1ToIntake1 = f.pathBuilder()
                .addPath(new BezierLine(poses.ALIGN1_POSE, poses.INTAKE1_POSE))
                .setLinearHeadingInterpolation(poses.ALIGN1_POSE.getHeading(), poses.INTAKE1_POSE.getHeading())
                .build();

        intake1ToLaunch = f.pathBuilder()
                .addPath(new BezierLine(poses.INTAKE1_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.INTAKE1_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        launchToIntake2 = f.pathBuilder()
                .addPath(new BezierCurve(poses.LAUNCH_POSE, poses.INTAKE_CONTROL, poses.INTAKE2_POSE))
                .setLinearHeadingInterpolation(poses.LAUNCH_POSE.getHeading(), poses.INTAKE2_POSE.getHeading())
                .build();

        intake2ToLaunch = f.pathBuilder()
                .addPath(new BezierLine(poses.INTAKE2_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.INTAKE2_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        launchToIntake3 = f.pathBuilder()
                .addPath(new BezierCurve(poses.LAUNCH_POSE, poses.INTAKE_CONTROL, poses.INTAKE3_POSE))
                .setLinearHeadingInterpolation(poses.LAUNCH_POSE.getHeading(), poses.INTAKE3_POSE.getHeading())
                .build();

        intake3ToLaunch = f.pathBuilder()
                .addPath(new BezierLine(poses.INTAKE3_POSE, poses.LAUNCH_POSE))
                .setLinearHeadingInterpolation(poses.INTAKE3_POSE.getHeading(), poses.LAUNCH_POSE.getHeading())
                .build();

        launchToIntake4 = f.pathBuilder()
                .addPath(new BezierLine(poses.LAUNCH_POSE, poses.INTAKE4_POSE))
                .setLinearHeadingInterpolation(poses.LAUNCH_POSE.getHeading(), poses.INTAKE4_POSE.getHeading())
                .build();

        end = new Path(new BezierLine(poses.INTAKE4_POSE, poses.END_POSE));
    }

    @Override
    public void loop() {
        f.update();
        sendPose();
        autoPathUpdates();
        hoodAngle = manualAngle ? angleHood : regressedAngle();
        if (hoodAngle > .65)
            hoodAngle = .65;
        if (hoodAngle < .25)
            hoodAngle = .25;
        midas.setHoodAngle(hoodAngle);
        trackingWithOdo(true);
        trackingWithTag(true);

        if (launching) {
            midas.setLaunchVelocity(regressedPower());
            //midas.spinToShoot();

        }

    }

    @Override
    public void stop() {

    }

    public void senseTag(boolean searchingID) {
        result = limelight.getLatestResult();
        if (searchingID) {
            try {
                telemetry.addData("ID", result.getFiducialResults().get(0).getFiducialId());
                ID = result.getFiducialResults().get(0).getFiducialId();
            } catch (Exception e) {
                telemetry.addData("ID", "not found");
            }
        } else {
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

    public void sendPose() {
        SharedData.toTeleopPose = f.getPose();
    }

    public double regressedAngle() {
        return (0.00210679 * calculateDist()) + (0.000785782 * midas.getLaunchVelocity()) + (-0.00000240321 * calculateDist() * midas.getLaunchVelocity()) - 0.578298;
    }

    public double regressedPower() {
        return (0.0287961 * calculateDist() * calculateDist()) + (-0.847604 * calculateDist()) + 1199.26456;
        //return (4.98089 * calculateDist())+918.9466;
        //return (4.89581 * calculateDist())+938.18835;
    }

    public double calculateDist() {
        return Math.sqrt(Math.pow(Math.abs(f.getPose().getX() - 0), 2) + Math.pow(Math.abs(f.getPose().getY() - 144), 2));
    }

    private void trackingWithTag(boolean usingTag) {
        if (usingTag) {
            senseTag(false);
            if (Math.abs(tX) > 1) {
                if (Math.abs(midas.getTurretPos() - midas.getTurretTargetPos()) < 120)
                    tagTicks = midas.getTurretPos() - (int) (5.771 * tX);
                if (tagTicks >= 2400) {
                    tagTicks = tagTicks - 2077;
                } else if (midas.getTurretTargetPos() <= -775) {
                    tagTicks = tagTicks + 2077;

                }
                midas.setTurretTargetPos(tagTicks);
            } else {
                if (aprilTagDetected)
                    midas.setTurretTargetPos(tagTicks);
                telemetry.addData("currently", "tag");

            }
        }
    }
    private void trackingWithOdo(boolean usingOdo) {
        if (usingOdo){

            double diffX = Math.abs(0 - f.getPose().getX());
            double diffY = Math.abs(144 - f.getPose().getY());
            telemetry.addData("theta" , ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            double theta = ((midas.getTurretPos() < 1000 ? -f.getHeading()*180/Math.PI : -f.getHeading()*180/Math.PI+360) + 90 + ((SharedData.side == Side.RED ? -1 : 1)*(Math.atan2(diffX, diffY)*180/Math.PI)));
            telemetry.addData("angle", theta);
            int odoTicks =  (int) ((theta)*5.771);

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

}
