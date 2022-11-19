package org.firstinspires.ftc.teamcode;

public class vec3 {
    double X, Y, Z;

    public vec3(double a) {
        X = a;
        Y = a;
        Z = a;
    }

    public vec3(double x, double y, double z) {
        X = x;
        Y = y;
        Z = z;
    }

    public vec3 set(double a) {
        X = a;
        Y = a;
        Z = a;

        return this;

    }

    public vec3 set(vec3 a) {
        X = a.X;
        Y = a.Y;
        Z = a.Z;

        return this;

    }

    public vec3 set(double x, double y, double z) {
        X = x;
        Y = y;
        Z = z;

        return this;
    }

    public vec3 add(double a, double b, double c) {
        X += a;
        Y += b;
        Z += c;

        return this;
    }

    public vec3 add(vec3 a) {
        add(a.X, a.Y, a.Z);

        return this;
    }

    public vec3 plus(double a, double b, double c) {
        return new vec3(X + a, Y + b, Z + c);
    }

    public vec3 plus(vec3 a) {
        return plus(a.X, a.Y, a.Z);
    }

    public vec3 neg() {
        return new vec3(-X, -Y, -Z);
    }

    public vec3 mul(double a) {
        return new vec3(X * a, Y * a, Z * a);
    }

    public vec3 abs() {
        return new vec3(Math.abs(X), Math.abs(Y), Math.abs(Z));
    }

    public double max() {
        return Math.max(X, Math.max(Y, Z));
    }

    public double min() {
        return Math.min(X, Math.min(Y, Z));
    }

    public double len2() {
        return X * X + Y * Y + Z * Z;
    }

    public double len() {
        return Math.sqrt(len2());
    }
}
