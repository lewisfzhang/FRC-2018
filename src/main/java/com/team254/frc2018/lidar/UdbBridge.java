package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;

import java.io.BufferedReader;
import java.io.InputStreamReader;

public class UdbBridge {
    private static Process chezyLidar = null;
    public static BufferedReader bufferedReader;



    public BufferedReader start() {
        if(chezyLidar == null) {
            System.out.println("Starting lidar");

            try {
                chezyLidar = new ProcessBuilder().command(Constants.kChezyLidarPath).start();
                InputStreamReader reader = new InputStreamReader( chezyLidar.getInputStream());
                return new BufferedReader(reader);
            } catch (Exception e) {
                System.err.println("Error: Could not start bridge");
                e.printStackTrace();
                return null;
            }
        } else {
            System.out.println("Error: bridge has already been started");
            return null;
        }
    }

    public boolean stop() {
        if(chezyLidar != null) {
            System.out.println("Stopping bridge");
            chezyLidar.destroyForcibly();
            return true;
        }
        System.out.println("Error: bridge not running");
        return false;
    }

}

