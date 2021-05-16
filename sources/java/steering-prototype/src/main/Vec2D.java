package main;

import java.util.Objects;
import java.util.Random;

public class Vec2D {

    public static final Vec2D ZERO = new Vec2D(0, 0);

    public static Vec2D randomUnitVec(Random rand) {
        double angle = rand.nextDouble() * 2 * Math.PI;
        return unitVec(angle);
    }

    public static Vec2D unitVec(double angle) {
        return new Vec2D(Math.cos(angle), Math.sin(angle));
    }

    public static Vec2D lerp(double frac, Vec2D low, Vec2D high) {
        return low.scale(1 - frac).add(high.scale(frac));
    }

    public static double invLerp(Vec2D lerped, Vec2D low, Vec2D high) {
        double fracX = (lerped.x - low.x) / (high.x - low.x);
        double fracY = (lerped.y - low.y) / (high.y - low.y);
        if(high.x == low.x) return fracY;
        if(high.y == low.y) return fracX;
        if(Math.abs(fracX - fracY) > 1e-8) {
            System.out.println(lerped);
            System.out.println(low);
            System.out.println(high);
            System.out.println(fracX);
            System.out.println(fracY);
            throw new IllegalArgumentException("arguments are not collinear");
        }
        return (fracX + fracY) / 2;
    }

    public final double x;
    public final double y;

    public Vec2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec2D add(Vec2D v) {
        return new Vec2D(x + v.x, y + v.y);
    }

    public Vec2D add(double x, double y) {
        return new Vec2D(this.x + x, this.y + y);
    }

    public Vec2D sub(Vec2D v) {
        return new Vec2D(x - v.x, y - v.y);
    }

    public Vec2D sub(double x, double y) {
        return new Vec2D(this.x - x, this.y - y);
    }

    public Vec2D scale(double factor) {
        return new Vec2D(x * factor, y * factor);
    }

    public double lengthSqr() {
        return x * x + y * y;
    }

    public double length() {
        return Math.sqrt(lengthSqr());
    }

    public double distSqr(Vec2D other) {
        return other.sub(this).lengthSqr();
    }

    public double dist(Vec2D other) {
        return Math.sqrt(distSqr(other));
    }

    public double angle() {
        double angle = Math.atan2(y, x);
        while(angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    public Vec2D rot(double angle) {
        return new Vec2D(x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle));
    }

    public Vec2D truncate(double maxLength) {
        return norm().scale(Math.min(length(), maxLength));
    }

    public double dot(Vec2D other) {
        return x * other.x + y * other.y;
    }

    public double cross(Vec2D other) {
        return (x * other.y) - (y * other.x);
    }

    public Vec2D perp() {
        return new Vec2D(y, -x);
    }

    public Vec2D norm() {
        if(equals(ZERO)) return ZERO;
        return scale(1 / length());
    }

    public double scalProj(Vec2D onto) {
        return dot(onto.norm());
    }

    public Vec2D vecProj(Vec2D onto) {
        return onto.scale(dot(onto) / onto.dot(onto));
    }

    public Vec2D vecRej(Vec2D from) {
        return sub(vecProj(from));
    }

    @Override
    public boolean equals(Object o) {
        if(this == o) return true;
        if(o == null || getClass() != o.getClass()) return false;
        Vec2D vec2D = (Vec2D) o;
        return Double.compare(vec2D.x, x) == 0 && Double.compare(vec2D.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
