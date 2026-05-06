package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.pedropathing.ivy.Command;

public interface Mechanism<S extends Enum<S>> {
    Command goTo(S state);

    S getState();

    void update();
}
