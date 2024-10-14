package teamcomm.data;


public class Contact {

    public enum ArmPushDirection {
        forward,
        backward,
        left,
        right,
        none,
    }

    public boolean armContact = false;
    public ArmPushDirection armPush = ArmPushDirection.left;
    public int lastArmContact;


    public Contact(boolean armContact, int armPush, int lastArmContact) {
        this.armContact = armContact;
        this.armPush = ArmPushDirection.values()[armPush];
        this.lastArmContact = lastArmContact;
    }

    // Override toString method to print the Contact
    @Override
    public String toString() {
        return "Contact {Arm=" + this.armContact + ", PushDirection=" + this.armPush + ", LastContact=" + this.lastArmContact + "}";
    }
}