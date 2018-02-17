package com.team254.frc2018.lidar.icp;

import java.util.Collection;

public class ReferenceModel {
    
    public Segment[] segments;
    
    public ReferenceModel(Segment... ss) {
        segments = ss;
    }
    
    public ReferenceModel(Collection<Segment> ss) {
        this(ss.toArray(new Segment[ss.size()]));
    }
    
    public Point getClosestPoint(Point p) {
        double minDist = Double.MAX_VALUE;
        Segment minSeg = null;
        for (Segment s : segments) {
            double dist = s.getDistanceSq(p);
            if (dist < minDist) {
                minDist = dist;
                minSeg = s;
            }
        }
        return minSeg.getClosestPoint(p);
    }
    
}