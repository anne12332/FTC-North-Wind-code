package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class vec2 {
    double X, Y;

    public vec2(double a) {
        X = a;
        Y = a;
    }

    public vec2(double x, double y) {
        X = x;
        Y = y;
    }

    public vec2 set(double a) {
        X = a;
        Y = a;

        return this;
    }

    public vec2 set(vec2 a) {
        X = a.X;
        Y = a.Y;

        return this;
    }

    public vec2 set(double x, double y) {
        X = x;
        Y = y;

        return this;
    }

    public vec2 add(double a, double b) {
        X += a;
        Y += b;

        return this;
    }

    public vec2 add(vec2 a) {
        add(a.X, a.Y);

        return this;
    }

    public vec2 plus(double a, double b) {
        return new vec2(X + a, Y + b);
    }

    public vec2 plus(vec2 a) {
        return plus(a.X, a.Y);
    }

    public vec2 neg() {
        return new vec2(-X, -Y);
    }

    public vec2 mul(double a) {
        return new vec2(X * a, Y * a);
    }

    public vec2 abs() {
        return new vec2(Math.abs(X), Math.abs(Y));
    }

    public double max() {
        return Math.max(X, Y);
    }

    public double min() {
        return Math.min(X, Y);
    }

    public double len2() {
        return X * X + Y * Y;
    }

    public vec2 turn(double ang){
        double x = X * cos(ang) + Y * sin(ang);
        double y = -X * sin(ang) + Y * cos(ang);
        return new vec2(x, y);
    }
}
