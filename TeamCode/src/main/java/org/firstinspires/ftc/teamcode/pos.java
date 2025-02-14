package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class pos {
    public static double[] clipStart = {782,20, 0}, clipEnd = {725, 0, 0};
    public static double[] firstBlockGrab = {987, -1075, 0}, secondBlockGrab = {420, -72, 0};
    public static double[] strafeToBucket1 = {560, -1300, 0}, strafeToBucket2 = {0, 0, 0};
    public static double clawLower = 1.1;
    public static double[] firstBucketCorrection = {115, 0, 0}, secondBucketCorrection = {145, 0, 0}, finalHang = {1200, 400, -75};

    public static double servohang1 = 0.45, servohang2 = 0.53;

    public  static double xOff, yOff;
}