package com.team254.frc2018.lidar;

import com.team254.lib.util.CircularBufferGeneric;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

/**
receives polar coordinates from the lidar scan and changes it into x,y coordinates,
also using the position and angle of the robot.
Stores Translation2d(basically a vector) in a Circular Buffer of Translation2d's.*/
public class LidarInterface {
	public  CircularBufferGeneric<Translation2d> lidarStorage = new CircularBufferGeneric<Translation2d>(400);
/**
 * @param scan the LidarScan with the data about the polar coordinates
 * @param pose the position and angle of the robot
 * Adds the scan to the array
 */
	public void addScan(LidarScan scan, RigidTransform2d pose) {
		double bXPos, bYPos, distance, xPos, yPos;
		bXPos = pose.getTranslation().x();
		bYPos = pose.getTranslation().y();
		Rotation2d lidarAngle = Rotation2d.fromDegrees(scan.angle);
		distance = scan.distance;
		Rotation2d compositeAngle = pose.getRotation().rotateBy(lidarAngle);
		xPos= (compositeAngle.cos()*distance)+bXPos;
		yPos= (compositeAngle.sin()*distance)+bYPos;
		//rounds to four decimal places
		xPos=Math.floor(xPos*10000)/10000;
		yPos=Math.floor(yPos*10000)/10000;
		Translation2d lidarPosition = new Translation2d(xPos,yPos);
		lidarStorage.addValue(lidarPosition);
	}
	public Translation2d[] getData() {
		return  lidarStorage.getLinkedList().toArray(new Translation2d[lidarStorage.getLinkedList().size()]);
	}
}