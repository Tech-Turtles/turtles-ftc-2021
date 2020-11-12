package org.firstinspires.ftc.teamcode.test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.ListIterator;

public class MenuCategory<T> {

    MenuOption<T> value;
    String name;
    ArrayList<MenuOption<T>> values;
    ListIterator<MenuOption<T>> list;

    MenuCategory(String name, MenuOption<T>... args) {
        this.name = name;

        values = new ArrayList<>();
        Collections.addAll(values, args);

        list = values.listIterator();
    }

    public void addOption(MenuOption<T> option) {
        values.add(option);
        list = values.listIterator(); // Test if it's necessary to make a new reference to the array
    }

    public MenuOption<T> selected() {
        return value;
    }

    public MenuOption<T> next() {
        if(list.hasNext()) {
            value = list.next();
        }
        return value;
    }

    public MenuOption<T> prev() {
        if(list.hasPrevious()) {
            value = list.previous();
        }
        return value;
    }

    public void apply() {
        for (MenuOption<T> menuOption : values) {
            menuOption.apply();
        }
    }

    @Override
    public String toString() {
        return name;
    }
}
