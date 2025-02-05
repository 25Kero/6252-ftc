package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class pos {
    public static double[] clipStart = {779,-30, 0}, clipEnd = {650, -45, 0};
    public static double[] firstBlockGrab = {880, -880, 0}, secondBlockGrab = {420, -72, 0};
    public static double[] strafeToBucket1 = {425, -1125, 0}, strafeToBucket2 = {0, 0, 0};
    public static double clawLower = 1.1, endLower = 0.85;
    public static double[] firstBucketCorrection = {115, 0, 0}, secondBucketCorrection = {145, 0, 0}, finalHang = {1200, 400, -75};

    public static double servohang1 = 0.45, servohang2 = 0.53;

    public  static double xOff, yOff;
}