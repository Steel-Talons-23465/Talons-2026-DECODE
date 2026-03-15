package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;

public class Midas {
    public static DcMotorEx turret, sorter, intake, launch;
    public static Servo adjustLeft, adjustRight;
    //make follower in here maybe

    public void initialize(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class , "turret");
        sorter = hardwareMap.get(DcMotorEx.class , "sorter");
        intake = hardwareMap.get(DcMotorEx.class , "intake");
        launch = hardwareMap.get(DcMotorEx.class , "launch");
        adjustLeft = hardwareMap.get(Servo.class , "adjustLeft");
        adjustRight = hardwareMap.get(Servo.class , "adjustRight");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void setHoodAngle(double position) {
        adjustLeft.setPosition(position); //0 -.6
        adjustRight.setPosition(1 - position); //.25-1
    }
    public void setLaunchVelocity(double velocity){
        launch.setVelocity(velocity);
    }


    public double getLaunchVelocity() {
        return launch.getVelocity();
    }

    public void setTurretTargetPos(int target){
        turret.setTargetPosition(target);
    }
    public int getTurretTargetPos(){
        return turret.getTargetPosition();
    }
    public int getTurretPos(){
        return turret.getCurrentPosition();
    }
}
