package org.firstinspires.ftc.teamcode;

public class ButtonToggle {
    public boolean toggleState = false;
    private boolean buttonIsActive = false;

    boolean toggled(boolean buttonPressed) {

        if(buttonPressed && (buttonIsActive == false)) {
            buttonIsActive = true;
            toggleState = !toggleState;
            return true;
        }
        else if(!buttonPressed) {
            buttonIsActive = false;
        }
        return false;
    }
}
