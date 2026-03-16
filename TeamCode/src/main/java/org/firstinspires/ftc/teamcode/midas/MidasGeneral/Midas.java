package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import android.text.method.Touch;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.MidasConstants;

public class Midas {
    public static DcMotorEx turret, sorter, intake, launch;
    public static Servo adjustLeft, adjustRight;
    //make follower in here maybe
    int slotGoal;
    public static TouchSensor magnet;

    public void initialize(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class , "turret");
        //sorter = hardwareMap.get(DcMotorEx.class , "sorter");
        intake = hardwareMap.get(DcMotorEx.class , "intake");
        launch = hardwareMap.get(DcMotorEx.class , "launch");
        adjustLeft = hardwareMap.get(Servo.class , "adjustLeft");
        adjustRight = hardwareMap.get(Servo.class , "adjustRight");
        magnet = hardwareMap.get(TouchSensor.class , "magnetic");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setPower(.5);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        sorter.setPower(1);
//        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
    public void startIntake(boolean intaking){
        intake.setPower(intaking ? 1 : -1);
    }
    public void stopIntake(){
        intake.setPower(0);
    }
//    public void spinToShoot(){sorter.setTargetPosition(-1425/*opposite direction one full rotation*/);}
//    public void sorterSPIIIN(boolean toShoot){
//        sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        sorter.setPower(toShoot ? 1 : -1);
//    }
//    public void stopSorter(){
//        sorter.setPower(0);
//    }
//    public void resetSorter(){
//        if (magnet.isPressed()){
//            sorter.setPower(0);
//            sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            sorter.setPower(1);
//            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//    }
//    public void setStoragePos(int slot){
//        int ticks = 1425;
//        if (slot == 0){
//            sorter.setTargetPosition(ticks/2);
//        }
//        else if (slot == 1){
//            sorter.setTargetPosition(ticks/3);
//        }
//        else if (slot == 2){
//            sorter.setTargetPosition(2*ticks/3);
//        }
//        resetSorter();
//    }

}
