package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;

public class UdbBridge {
    private static Process chezyLidar = null;

    public boolean start() {
        if(chezyLidar == null) {
            System.out.println("Starting lidar");
            Runtime r = Runtime.getRuntime();

            try {
                chezyLidar = r.exec(Constants.kChezyLidarPath);
                chezyLidar.waitFor();
            } catch (Exception e) {
                System.err.println("Error: Could not start bridge");
                e.printStackTrace();
                return false;
            }
            return true;
        } else {
            System.out.println("Error: bridge has already been started");
            return false;
        }
    }

    public boolean stop() {
        if(chezyLidar != null) {
            System.out.println("Stopping bridge");
            chezyLidar.destroy();
            chezyLidar = null;
            return true;
        }
        System.out.println("Error: bridge not running");
        return false;
    }

}
