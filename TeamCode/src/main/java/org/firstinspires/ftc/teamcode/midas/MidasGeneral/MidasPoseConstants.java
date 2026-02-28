package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;

public class MidasPoseConstants {
    public Pose goal = SharedData.side == Side.RED ? new Pose(136, 12, Math.toRadians(145)) : new Pose(12,  136, Math.toRadians(35)) ;



}
