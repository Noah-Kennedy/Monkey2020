package main;

public final class Util {

    public static double clamp(double value, double low, double high) {
        return Math.min(Math.max(low, value), high);
    }
}
