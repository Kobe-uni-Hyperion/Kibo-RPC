package jp.jaxa.iss.kibo.rpc.defaultapk;

public final class BoundingBox {
    private final float x1;
    private final float y1;
    private final float x2;
    private final float y2;
    private final float cx;
    private final float cy;
    private final float w;
    private final float h;
    private final float cnf;
    private final int cls;
    private final String clsName;

    public BoundingBox(float x1, float y1, float x2, float y2, float cx, float cy, float w, float h, float cnf, int cls, String clsName) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.cx = cx;
        this.cy = cy;
        this.w = w;
        this.h = h;
        this.cnf = cnf;
        this.cls = cls;
        this.clsName = clsName;
    }

    public float getX1() {
        return x1;
    }

    public float getY1() {
        return y1;
    }

    public float getX2() {
        return x2;
    }

    public float getY2() {
        return y2;
    }

    public float getCx() {
        return cx;
    }

    public float getCy() {
        return cy;
    }

    public float getW() {
        return w;
    }

    public float getH() {
        return h;
    }

    public float getCnf() {
        return cnf;
    }

    public int getCls() {
        return cls;
    }

    public String getClsName() {
        return clsName;
    }

    public BoundingBox copy(float x1, float y1, float x2, float y2, float cx, float cy, float w, float h, float cnf, int cls, String clsName) {
        return new BoundingBox(x1, y1, x2, y2, cx, cy, w, h, cnf, cls, clsName);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof BoundingBox)) return false;
        BoundingBox that = (BoundingBox) o;
        return Float.compare(that.x1, x1) == 0 &&
                Float.compare(that.y1, y1) == 0 &&
                Float.compare(that.x2, x2) == 0 &&
                Float.compare(that.y2, y2) == 0 &&
                Float.compare(that.cx, cx) == 0 &&
                Float.compare(that.cy, cy) == 0 &&
                Float.compare(that.w, w) == 0 &&
                Float.compare(that.h, h) == 0 &&
                Float.compare(that.cnf, cnf) == 0 &&
                cls == that.cls &&
                clsName.equals(that.clsName);
    }

    @Override
    public int hashCode() {
        int result = (x1 != +0.0f ? Float.floatToIntBits(x1) : 0);
        result = 31 * result + (y1 != +0.0f ? Float.floatToIntBits(y1) : 0);
        result = 31 * result + (x2 != +0.0f ? Float.floatToIntBits(x2) : 0);
        result = 31 * result + (y2 != +0.0f ? Float.floatToIntBits(y2) : 0);
        result = 31 * result + (cx != +0.0f ? Float.floatToIntBits(cx) : 0);
        result = 31 * result + (cy != +0.0f ? Float.floatToIntBits(cy) : 0);
        result = 31 * result + (w != +0.0f ? Float.floatToIntBits(w) : 0);
        result = 31 * result + (h != +0.0f ? Float.floatToIntBits(h) : 0);
        result = 31 * result + (cnf != +0.0f ? Float.floatToIntBits(cnf) : 0);
        result = 31 * result + cls;
        result = 31 * result + clsName.hashCode();
        return result;
    }

    @Override
    public String toString() {
        return "BoundingBox(x1=" + x1 + ", y1=" + y1 + ", x2=" + x2 + ", y2=" + y2 + ", cx=" + cx + ", cy=" + cy + ", w=" + w + ", h=" + h + ", cnf=" + cnf + ", cls=" + cls + ", clsName=" + clsName + ")";
    }
}

