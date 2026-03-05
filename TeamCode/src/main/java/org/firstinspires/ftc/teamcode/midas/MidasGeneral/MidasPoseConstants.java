package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;

public class MidasPoseConstants {
    public Pose goal = SharedData.side == Side.RED ? new Pose(144, 144, Math.toRadians(145)) : new Pose(0,  144, Math.toRadians(35)) ;

    public Pose StartPose = SharedData.side != Side.RED ? new Pose(56.75,9.36, Math.toRadians(90)) : new Pose(87.25,8.75, Math.toRadians(90));

}
