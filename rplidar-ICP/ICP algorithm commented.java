public Transform doICP(ReferenceModel reference, int iterations) {
    debug("Doing ICP registration ("+iterations+" iters)...");
    long startTime = System.nanoTime(); // store the timestamp when we started
    
    // This variable stores the current transformation that
    // we think makes the reference fit our data most closely.
    // Initialize to some guess; in this case, a global variable
    // that lets us single-step the algorithm over multiple
    // calls to this method.
    Transform trans = icpTrans;
    
    // do the ICP process however many times were requested
    for (int n = 0; n < iterations+1; n++) {
        ////// find pairs of corresponding points //////
        //  (and some values used in the next step)
        
        // cache the inverse of the current transformation
        // (this is used in finding point pairs)
        Transform transInv = trans.inverse();
        
        // clear the list of corresponding point-pairs
        pairs.clear();
        
        // these variables hold sums of various terms;
        // they're used in computing the new transform
        double SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
        double Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
        
        // loop over each LIDAR point
        for (Point p : points) {
            // given the current transform, find the point on the reference model
            // that's closest to the current LIDAR point
            Point p2 = transInv.apply(p); // get the (inverse-)transformed point
            Point rp = reference.getClosestPoint(p2); // the corresponding (closest) point on the reference
            
            // if the point is too far away from the reference model,
            // mark it as "not good" and skip it (it's probably an outlier)
            // (compare the squared distance, so we don't have to do a sqrt)
            p.good = p2.getDistanceSq(rp) < OUTLIER_THRESH*OUTLIER_THRESH;
            if (!p.good) continue; // on to the next point!
            
            // add the corresponding point-pair to our list
            // (note that this list is only needed for the visualization,
            // because computing the new transform only requires the
            // terms that are summed up next, inside this loop)
            pairs.add(new PointPair(p, rp));
            
            // sum up these terms for use in the next step...
            SumXa += p.x;
            SumYa += p.y;
            
            SumXb += rp.x;
            SumYb += rp.y;
            
            Sxx += p.x * rp.x;
            Sxy += p.x * rp.y;
            Syx += p.y * rp.x;
            Syy += p.y * rp.y;
        }
        
        // This thingy is just for the purposes of the visualization:
        // it cuts the last iteration short, so that the pairs are
        // computed (above) but not the next transform (below).
        // (so we can see the current matched pairs before the
        // transformation is applied)
        if (n==iterations) break;
        
        ////// calculate the new transform //////
        
        final int N = pairs.size();
        if (N==0) {
            // If there are zero points, then we'll end up dividing
            // by zero (and it doesn't make sense to find a fit for
            // zero data points, anyway...)
            // This could be because the input list of points is zero,
            // or because all the points were skipped as "outliers."
            // TODO: handle this better, or avoid it entirely
            return new Transform();
        }
        final double N_inv = 1.0 / N; // cache this because multiplication is faster than division
        
        // compute the means of the various coordinates
        final double mean_x_a = SumXa * N_inv;
        final double mean_y_a = SumYa * N_inv;
        final double mean_x_b = SumXb * N_inv;
        final double mean_y_b = SumYb * N_inv;
        
        // Auxiliary variables Ax,Ay  (fan-C math ¯\_(ツ)_/¯)
        final double Ax = N * (Sxx + Syy) - SumXa * SumXb - SumYa * SumYb;
        final double Ay = SumXa * SumYb + N * (Syx - Sxy) - SumXb * SumYa;
        
        // the rotation angle for the new transform
        final double theta = (Ax == 0 && Ay == 0)? 0.0 : Math.atan2(Ay, Ax);
        
        // cache the sine and cosine of the angle
        final double ccos = Math.cos(theta);
        final double csin = Math.sin(theta);
        
        // the translation part of the new transform  (fan-C math ¯\_(ツ)_/¯)
        final double tx = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
        final double ty = mean_y_a - mean_x_b * csin - mean_y_b * ccos;
        
        if (theta==trans.theta && tx==trans.tx && ty==trans.ty) {
            // the transform is literally unchanged from the previous
            // iteration, so we've definitely converged
            debug("Converged on iteration n="+n);
            break; // exit the loop
        }
        
        // update our variable to the new transform we've computed
        trans = new Transform(theta, tx, ty, csin, ccos);
    }
    
    long endTime = System.nanoTime(); // the timestamp when we finished
    debug("Done ("+((endTime-startTime)/1000000)+" ms)"); // divide by 1,000,000 to convert ns to ms
    debug(trans); // print the final computed transform just for kicks
    
    // set these global variables, which are used for the visualization
    icpTrans = trans;
    transReference = trans.apply(reference);
    return trans;
}