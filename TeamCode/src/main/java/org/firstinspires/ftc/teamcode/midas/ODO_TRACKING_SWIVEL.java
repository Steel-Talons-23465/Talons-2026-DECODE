package org.firstinspires.ftc.teamcode.midas;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hornet.General.PoseConstants;
import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;
import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;

public class ODO_TRACKING_SWIVEL extends LinearOpMode {

    private Follower f;
    private PoseConstants poses = new PoseConstants();
    private DcMotorEx turret = null;
    private double goalAngle, delta, speedMultiplier, adjustAngle, angleVel, tX, tY;
    private Vector vel = new Vector(new Pose(0,0,0));
    private int deltaTicks, currentPos, targetPos;
    private Limelight3A limelight = null;
    private LLResult result;
    private boolean tagTracking;


    public void runOpMode() throws InterruptedException{
        f = MidasConstants.createFollower(hardwareMap);
        turret = hardwareMap.get(DcMotorEx.class , "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.75);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        f.setStartingPose(poses.START_POSE);

        waitForStart();
        f.startTeleOpDrive(true);

        while (opModeIsActive()){
            f.update();

            angleVel = f.getAngularVelocity();
            vel = f.getVelocity();
            updateMovement();

            goalAngle = SharedData.side == Side.RED ? Math.toRadians(35) : Math.toRadians(145) ;

            telemetry.addData("AngVel", f.getAngularVelocity());
            telemetry.addData("Vel" , f.getVelocity());
            telemetry.update();

            double turnLimit = Math.toRadians(goalAngle + 180);
            if (f.getHeading() <= turnLimit) {
                delta = -Math.abs(f.getHeading() - goalAngle);
            }
            else if (f.getHeading() > turnLimit) {
                delta = Math.abs(f.getHeading() - goalAngle);
            }
            else delta = 0;
            //if we're between shooting poses, set turret stop;

            deltaTicks = (int) (delta*5.771);
            turret.setTargetPosition(currentPos + deltaTicks);

            if (Math.signum(angleVel) >= 0){
                 delta -= angleVel/5.771;
            }
            else if (Math.signum(angleVel) < 0){
                delta += angleVel/5.771;
            }
            deltaTicks =  (int) (delta*5.771);
            turret.setTargetPosition(currentPos + deltaTicks);

            if (gamepad1.aWasPressed()){
                tagTracking = !tagTracking;
            }
            if (Math.abs(tX) > 1 && tagTracking){
                targetPos = turret.getCurrentPosition() - (int)(5.771*tX);
                turret.setTargetPosition(targetPos);
            }
            else {
                targetPos = turret.getTargetPosition();
                turret.setTargetPosition(targetPos);
            }




        }

        }
        private void updateMovement(){

            f.setTeleOpDrive(
                    -gamepad1.left_stick_y * speedMultiplier,
                    -gamepad1.left_stick_x * speedMultiplier,
                    -gamepad1.right_stick_x * speedMultiplier,
                    true,
                    0);

            if(gamepad1.left_trigger >= .2)
                speedMultiplier = .2;
            else
                speedMultiplier = 1;
        }


    public void senseTag(){
        result = limelight.getLatestResult();

        try {
            tX = result.getFiducialResults().get(0).getTargetXDegrees();
            tY = result.getFiducialResults().get(0).getTargetYDegrees();
            telemetry.addData("Reading Apriltag" , result.getFiducialResults().get(0).getFiducialId() );
            telemetry.addData("TagX" , tX);
            telemetry.addData("TagY" , tY);

        }
        catch (Exception e){
            tX = 0;
            tY = 0;
            telemetry.addData("No AprilTag Detected" , "-1");
        }

    }





}
