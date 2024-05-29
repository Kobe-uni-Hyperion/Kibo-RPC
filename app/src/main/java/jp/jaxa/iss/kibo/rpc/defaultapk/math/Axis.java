package jp.jaxa.iss.kibo.rpc.defaultapk.math;

import gov.nasa.arc.astrobee.types.Vec3d;

public enum Axis {
    XP(1,0,0),
    XN(-1,0,0),
    YP(0,1,0),
    YN(0,-1,0),
    ZP(0,0,1),
    ZN(0,0,-1);

    private final Vec3d vec;
    private final float x,y,z;

    Axis(float x,float y,float z){
        this.x=x;
        this.y=y;
        this.z=z;
        this.vec=new Vec3d(x,y,z);
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    public Vec3d asVec3d() {
        return vec;
    }
}
