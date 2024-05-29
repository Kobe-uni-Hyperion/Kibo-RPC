package jp.jaxa.iss.kibo.rpc.defaultapk.math;

import gov.nasa.arc.astrobee.types.Vec3d;

public class VecUtil {
    private VecUtil(){}
    public static Vec3d add(Vec3d vec1,Vec3d vec2){
        double[] v1= vec1.toArray();
        double[] v2= vec2.toArray();
        return new Vec3d(v1[0]+v2[0],v1[1]+v2[1],v1[2]+v2[2]);
    }

    public static double dotProduct(Vec3d vec1,Vec3d vec2){
        double[] v1= vec1.toArray();
        double[] v2= vec2.toArray();
        return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
    }

    public static Vec3d crossProduct(Vec3d vec1,Vec3d vec2){
        double[] v1= vec1.toArray();
        double[] v2= vec2.toArray();
        return new Vec3d(
                v1[1]*v2[2]-v1[2]*v2[1],
                v1[2]*v2[0]-v1[0]*v2[2],
                v1[0]*v2[1]-v1[1]*v2[0]
        );
    }

    public static Vec3d inverse(Vec3d vec){
        double[] v= vec.toArray();
        return new Vec3d(-v[0],-v[1],-v[2]);
    }
}
