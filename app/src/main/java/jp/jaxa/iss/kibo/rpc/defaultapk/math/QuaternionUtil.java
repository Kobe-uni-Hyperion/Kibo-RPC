package jp.jaxa.iss.kibo.rpc.defaultapk.math;

import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.types.Vec3d;

public class QuaternionUtil{
    private QuaternionUtil(){}
    private static Quaternion rotateAlongNormalizedVec(float axisX,float axisY, float axisZ,float rotationAmount){
        float sin= (float) Math.sin(rotationAmount/2);
        return new Quaternion(
                axisX*sin,
                axisY*sin,
                axisZ*sin,
                (float) Math.cos(rotationAmount/2)
        );
    }

    /**
     * 回転を表す四元数を返します
     * @param axisX 回転軸ベクトルのXの値
     * @param axisY 回転軸ベクトルのYの値
     * @param axisZ 回転軸ベクトルのZの値
     * @param rotationAmount 回転量[rad]
     */
    public static Quaternion rotate(float axisX,float axisY, float axisZ,float rotationAmount){
        float length= (float) Math.sqrt(axisX*axisX+axisY*axisY+axisZ*axisZ);
        return rotateAlongNormalizedVec(axisX/length, axisY/length, axisZ/length, rotationAmount);
    }
    /**
     * 回転を表す四元数を返します
     * @param axis 回転軸ベクトル
     * @param rotationAmount 回転量[rad]
     */
    public static Quaternion rotate(Vec3d axis,float rotationAmount){
        double[] axisValues=axis.toArray();
        return rotate((float) axisValues[0], (float) axisValues[1], (float) axisValues[2],rotationAmount);
    }
    /**
     * 回転を表す四元数を返します
     * @param axis 回転軸
     * @param rotationAmount 回転量[rad]
     */
    public static Quaternion rotate(Axis axis,float rotationAmount){
        return rotateAlongNormalizedVec(axis.getX(),axis.getY(),axis.getZ(),rotationAmount);
    }

    /**
     * 二つの四元数についてハミルトン積を計算します
     * @return 二数のハミルトン積 (q1*q2)
     */
    public static Quaternion product(Quaternion q1,Quaternion q2){
        float w1=q1.getW(),x1=q1.getX(),y1=q1.getY(),z1=q1.getZ();
        float w2=q2.getW(),x2=q2.getX(),y2= q2.getY(),z2=q2.getZ();
        return new Quaternion(
                w1*x2+w2*x1+y1*z2-y2*z1,
                w1*y2+w2*y1-x1*z2+x2*z1,
                w1*z2+w2*z1+x1*y2-x2*y1,
                w1*w2-x1*x2-y1*y2-z1*z2
        );
    }

    /**
     * 与えられた四元数と共役な値を返します
     * @param q 与える四元数
     * @return 入力に共役な四元数
     */
    public static Quaternion getConjugateValueOf(Quaternion q){
        return new Quaternion(-q.getX(), -q.getY(), -q.getZ(), q.getW());
    }
}
