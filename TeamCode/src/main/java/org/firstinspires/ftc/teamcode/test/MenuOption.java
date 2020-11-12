package org.firstinspires.ftc.teamcode.test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ListIterator;

public class MenuOption<T> {

    T value;
    String name;
    MenuObject<T> menuObject;
    ListIterator<T> list;

    MenuOption(MenuObject<T> menuObject, String name, T... args) {
        this.menuObject = menuObject;
        this.name = name;

        ArrayList<T> values = new ArrayList<>();
        Collections.addAll(values, args);

        list = values.listIterator();
    }

    public T selected() {
        return value;
    }

    public T next() {
        value = list.hasNext() ? list.next() : value;
        return value;
    }

    public T prev() {
        value = list.hasPrevious() ? list.previous() : value;
        return value;
    }

    public void apply() {
        menuObject.set(value);
    }

    @Override
    public boolean equals(Object obj) {
        return menuObject.equals(obj);
    }

    @Override
    public String toString() {
        return name;
    }
}
