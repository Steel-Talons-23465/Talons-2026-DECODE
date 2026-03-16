package org.firstinspires.ftc.teamcode.midas.MidasGeneral;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hornet.General.SharedData;
import org.firstinspires.ftc.teamcode.hornet.General.Side;

public class MidasPoseConstants {
    public Pose goal = SharedData.side == Side.RED ? new Pose(144, 144, Math.toRadians(145)) : new Pose(0,  144, Math.toRadians(35)) ;

    public Pose START_POSE = SharedData.side != Side.RED ? new Pose(57,9, Math.toRadians(90)) : new Pose(87,9, Math.toRadians(90));
    public Pose LAUNCH_POSE = SharedData.side != Side.RED ? new Pose(57, 18, Math.toRadians(90)) : new Pose(87,18,Math.toRadians(0));
    public Pose ALIGN1_POSE = SharedData.side != Side.RED ? new Pose(18,9, Math.toRadians(180)) : new Pose(126, 9, Math.toRadians(0));
    public Pose INTAKE1_POSE = SharedData.side != Side.RED ? new Pose(9,9,Math.toRadians(180)): new Pose(135,9,Math.toRadians(0));
    public Pose INTAKE2_POSE = SharedData.side != Side.RED ? new Pose(24, 27 , Math.toRadians(90)) : new Pose(0,0,0);
    public Pose INTAKE3_POSE = SharedData.side != Side.RED ? new Pose(24, 51, Math.toRadians(180)) : new Pose();
    public Pose INTAKE_CONTROL = SharedData.side != Side.RED ? new Pose(23, 9) : new Pose();
    public Pose INTAKE4_POSE = SharedData.side != Side.RED ? new Pose(16, 24, Math.toRadians(135)) : new Pose();

    public Pose END_POSE = SharedData.side != Side.RED ? new Pose() : new Pose();
}
