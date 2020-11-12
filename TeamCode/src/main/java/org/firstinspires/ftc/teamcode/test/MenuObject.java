package org.firstinspires.ftc.teamcode.test;

public class MenuObject<T> {
    private T var;

    public MenuObject(T initial)
    {
        this.var = initial;
    }

    public T get()
    {
        return var;
    }

    public void set(T var)
    {
        this.var = var;
    }

    public String toString()
    {
        return var.toString();
    }
}
