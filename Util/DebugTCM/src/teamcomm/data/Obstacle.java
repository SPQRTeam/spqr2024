package teamcomm.data;


public class Obstacle {

    public static final int MAX_OBS_SIZE = 20; // set this accordingly to MAX_OBSTACLES in /Tools/Communication/DebugMessageTCM.h

    public enum ObsTypes {
        goalpost,
        unknown,
        someRobot,
        opponent,
        teammate,
        fallenSomeRobot,
        fallenOpponent,
        fallenTeammate,
    }

    public ObsTypes type = ObsTypes.unknown;
    public float[] center = new float[2];
    public int lastSeen;


    public Obstacle(int type, float[] center, int lastSeen) {
        this.type = ObsTypes.values()[type];
        for(int i = 0; i < 2; i++) {
            this.center[i] = center[i];            
        }
        this.lastSeen = lastSeen;
    }

    // Override toString method to print the obstacle
    @Override
    public String toString() {
        return "Obstacle {type='" + this.type + "', Center='" + this.center[0] + " , " + this.center[1] + 
                "', LastSeen='" + this.lastSeen + "}";
    }
}