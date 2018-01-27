package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import com.team254.lib.util.math.Translation2d;

import java.util.ArrayList;

/**
 * Holds a single 360 degree scan from the lidar
 */
public class LidarScan {
    public ArrayList<Translation2d> points = new ArrayList<>(Constants.kScanSize);

    public String toJsonString() {
        String json = "{\"scan\": [";
        for(Translation2d point : points) {
            json += "{\"x\":" + point.x() + ", \"y\":" + point.y() + "},";
        }
        json = json.substring(0, json.length()-1);
        json += "]}";
        return json;
    }

    public String toString() {
        String s = "";
        for(Translation2d point : points) {
            s += "x: " + point.x() + ", y: " + point.y() + "\n";
        }
        return s;
    }

    public void addPoint(LidarPoint point) {
        points.add(point.toCartesian());
    }
}