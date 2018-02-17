package com.team254.frc2018.lidar.icp;

import java.util.Collection;

public class ICP {
    
    public static final double OUTLIER_THRESH = 300;
    
    public ReferenceModel reference;
    public long timeoutNs;
    
    public ICP(ReferenceModel ref, long timeoutMs) {
        reference = ref;
        timeoutNs = timeoutMs*1000000;
    }
    
    /**
     * Applies ICP point registration to find a Transform that aligns
     * the given point cloud with the reference model. The returned
     * Transform represents the 2D pose (translation, rotation) of the
     * LIDAR sensor in the reference's coordinate system.
     * <p>
     * A result is returned after either the algorithm converges or
     * it times out.
     * 
     * @param points The point cloud to align
     * @param trans An initial guess Transform (if null, the identity is used)
     * @return The computed Transform
     */
    public Transform doICP(Collection<Point> points, Transform trans) {
        long startTime = System.nanoTime();
        
        trans = trans==null? new Transform() : trans.inverse();
        while (System.nanoTime() - startTime < timeoutNs) {
            /// get pairs of corresponding points
            Transform transInv = trans.inverse();
            
            double SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
            double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
            int N = 0;
            for (Point p : points) {
                Point p2 = transInv.apply(p);
                Point rp = reference.getClosestPoint(p2);
                boolean isOutlier = p2.getDistanceSq(rp) > OUTLIER_THRESH*OUTLIER_THRESH;
                if (isOutlier) continue;
                N++;
                
                // Compute the terms:
                SumXa += p.x;
                SumYa += p.y;
                
                SumXb += rp.x;
                SumYb += rp.y;
                
                Sxx += p.x * rp.x;
                Sxy += p.x * rp.y;
                Syx += p.y * rp.x;
                Syy += p.y * rp.y;
            }
            
            // TODO: compute the mean & [variance|std. dev.] for the point distances,
            //       then reject outliers whose distance is more than mean+[some multiple of variance]
            // (also look up popular variants of ICP that deal with outliers)
            
            /// calculate the new transform
            // code based on http://mrpt.ual.es/reference/devel/se2__l2_8cpp_source.html#l00158
            if (N==0) throw new RuntimeException("ICP: no matching points"); // TODO: handle this better, or avoid it
            final double N_inv = 1.0 / N;
            
            final double mean_x_a = SumXa * N_inv;
            final double mean_y_a = SumYa * N_inv;
            final double mean_x_b = SumXb * N_inv;
            final double mean_y_b = SumYb * N_inv;
            
            // Auxiliary variables Ax,Ay:
            final double Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
            final double Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;
            
            final double theta = (Ax == 0 && Ay == 0)? 0.0 : Math.atan2(Ay, Ax);
            
            final double ccos = Math.cos(theta);
            final double csin = Math.sin(theta);
            
            final double tx = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
            final double ty = mean_y_a - mean_x_b * csin - mean_y_b * ccos;
            
            if (theta==trans.theta && tx==trans.tx && ty==trans.ty) {
                // converged; double values have stopped changing
                break;
            }
            trans = new Transform(theta, tx, ty, csin, ccos);
        }
        
        return trans.inverse();
    }
    
}