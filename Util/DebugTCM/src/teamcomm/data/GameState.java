package teamcomm.data;

import common.ApplicationLock;
import common.net.logging.Logger;
import data.GameControlData;
import data.GameControlReturnData;
import data.Rules;
import data.SPLTeamMessage;
import data.TeamInfo;
import data.Teams;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import javax.swing.event.EventListenerList;
import teamcomm.PluginLoader;
import teamcomm.data.event.GameControlDataEvent;
import teamcomm.data.event.GameControlDataEventListener;
import teamcomm.data.event.GameControlDataTimeoutEvent;
import teamcomm.data.event.TeamEvent;
import teamcomm.data.event.TeamEventListener;
import teamcomm.net.logging.LogReplayer;

import java.util.Arrays;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import teamcomm.data.Obstacle;
import teamcomm.data.Contact;

/**
 * Singleton class managing the known information about communicating robots.
 *
 * @author Felix Thielke
 */
public class GameState implements GameControlDataEventListener {

    /**
     * Index of the team playing on the left side of the field.
     */
    public static final int TEAM_LEFT = 0;
    /**
     * Index of the team playing on the right side of the field.
     */
    public static final int TEAM_RIGHT = 1;
    /**
     * Index of the virtual team containing illegally communicating robots.
     */
    public static final int TEAM_OTHER = 2;

    private static final int CHANGED_LEFT = 1;
    private static final int CHANGED_RIGHT = 2;
    private static final int CHANGED_OTHER = 4;

    private static final GameState instance = new GameState();

    private GameControlData lastGameControlData;

    private final int[] teamNumbers = new int[]{0, 0};

    private boolean mirrored = false;

    private final Map<Integer, Collection<RobotState>> robots = new HashMap<>();

    private static final Comparator<RobotState> playerNumberComparator = (o1, o2) -> {
        if (o1.getPlayerNumber() == null) {
            if (o2.getPlayerNumber() == null) {
                return o1.hashCode() - o2.hashCode();
            }
            return -1;
        } else if (o2.getPlayerNumber() == null) {
            return 1;
        }
        return o1.getPlayerNumber() - o2.getPlayerNumber();
    };

    private final HashMap<String, RobotState> robotsByAddress = new HashMap<>();

    private final EventListenerList listeners = new EventListenerList();

    private final ScheduledFuture<?> taskHandle;

    /**
     * Returns the only instance of the RobotData class.
     *
     * @return instance
     */
    public static GameState getInstance() {
        return instance;
    }

    private GameState() {
        ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
        taskHandle = scheduler.scheduleAtFixedRate(() -> {
            if (!(LogReplayer.getInstance().isReplaying() && LogReplayer.getInstance().isPaused())) {
                // Check if the GameController is running
                try {
                    final ApplicationLock lock = new ApplicationLock("GameController");
                    if (!lock.acquire()) {
                        // Do not log messages if a GameController is running on the same system
                        Logger.getInstance().disableLogging();
                    } else {
                        Logger.getInstance().enableLogging();
                        lock.release();
                    }
                } catch (IOException e) {
                }

                // Update robots
                int changed = 0;
                synchronized (robotsByAddress) {
                    robotsByAddress.values().removeIf(r -> r.updateConnectionStatus() == RobotState.ConnectionStatus.INACTIVE);

                    for (final Entry<Integer, Collection<RobotState>> team : robots.entrySet()) {
                        final Iterator<RobotState> it = team.getValue().iterator();
                        while (it.hasNext()) {
                            final RobotState r = it.next();
                            if (!robotsByAddress.containsKey(r.getAddress())) {
                                it.remove();

                                synchronized (teamNumbers) {
                                    if (team.getKey() == teamNumbers[TEAM_LEFT]) {
                                        changed |= CHANGED_LEFT;
                                        if (team.getValue().isEmpty() && lastGameControlData == null) {
                                            teamNumbers[TEAM_LEFT] = 0;
                                        }
                                    } else if (team.getKey() == teamNumbers[TEAM_RIGHT]) {
                                        changed |= CHANGED_RIGHT;
                                        if (team.getValue().isEmpty() && lastGameControlData == null) {
                                            teamNumbers[TEAM_RIGHT] = 0;
                                        }
                                    } else {
                                        changed |= CHANGED_OTHER;
                                    }
                                }
                            }
                        }
                    }
                }

                sendEvents(changed);
            }
        }, RobotState.ConnectionStatus.HIGH_LATENCY.threshold * 2L, RobotState.ConnectionStatus.HIGH_LATENCY.threshold / 2L, TimeUnit.MILLISECONDS);
    }

    /**
     * Shuts down the thread which removes inactive robots. To be called before
     * the program exits.
     */
    public void shutdown() {
        taskHandle.cancel(false);
    }

    /**
     * Resets all information about robots and teams.
     */
    public void reset() {
        lastGameControlData = null;
        synchronized (teamNumbers) {
            teamNumbers[0] = 0;
            teamNumbers[1] = 0;
        }
        synchronized (robotsByAddress) {
            robots.clear();
            robotsByAddress.clear();
        }
        sendEvents(CHANGED_LEFT | CHANGED_RIGHT | CHANGED_OTHER);
    }

    /**
     * Updates info about the game with a message from the GameController.
     *
     * @param e event containing the data sent by the GameController
     */
    @Override
    public void gameControlDataChanged(final GameControlDataEvent e) {
        int changed = 0;

        if (lastGameControlData == null) {
            synchronized (teamNumbers) {
                teamNumbers[TEAM_LEFT] = e.data.team[0].teamNumber;
                teamNumbers[TEAM_RIGHT] = e.data.team[1].teamNumber;
            }
            changed = CHANGED_LEFT | CHANGED_RIGHT | CHANGED_OTHER;
        } else {
            synchronized (teamNumbers) {
                if (e.data.team[0].teamNumber != teamNumbers[TEAM_LEFT]) {
                    teamNumbers[TEAM_LEFT] = e.data.team[0].teamNumber;
                    changed = CHANGED_LEFT | CHANGED_OTHER;
                }
                if (e.data.team[1].teamNumber != teamNumbers[TEAM_RIGHT]) {
                    teamNumbers[TEAM_RIGHT] = e.data.team[1].teamNumber;
                    changed |= CHANGED_RIGHT | CHANGED_OTHER;
                }
            }
        }

        // Update penalties
        for (final TeamInfo team : e.data.team) {
            final Collection<RobotState> teamRobots = robots.get((int) team.teamNumber);
            if (teamRobots != null) {
                for (final RobotState r : teamRobots) {
                    if (r.getPlayerNumber() != null && r.getPlayerNumber() <= team.player.length) {
                        r.setPenalty(team.player[r.getPlayerNumber() - 1].penalty);
                    }
                }
            }
        }

        if (changed != 0) {
            // (re)load plugins
            PluginLoader.getInstance().update((int) e.data.team[0].teamNumber, (int) e.data.team[1].teamNumber);
        }

        if (LogReplayer.getInstance().isReplaying()) {
            lastGameControlData = e.data;
            sendEvents(changed);
            return;
        }

        // Open a new logfile for the current GameController state if the
        // state changed from or to initial/finished
        final StringBuilder logfileName = new StringBuilder();
        if (e.data.firstHalf == GameControlData.C_TRUE) {
            logfileName.append(getTeamName((int) e.data.team[0].teamNumber, false, false)).append("_").append(getTeamName((int) e.data.team[1].teamNumber, false, false));
        } else {
            logfileName.append(getTeamName((int) e.data.team[1].teamNumber, false, false)).append("_").append(getTeamName((int) e.data.team[0].teamNumber, false, false));
        }
        if (!Rules.league.startWithPenalty) {
            logfileName.append(e.data.firstHalf == GameControlData.C_TRUE ? "_1st" : "_2nd").append("Half");
        }
        if (e.data.gameState == GameControlData.STATE_READY && (lastGameControlData == null || lastGameControlData.gameState == GameControlData.STATE_INITIAL)) {
            Logger.getInstance().createLogfile(logfileName.toString());
        } else if (e.data.gameState == GameControlData.STATE_INITIAL && (lastGameControlData == null || lastGameControlData.gameState != GameControlData.STATE_INITIAL)) {
            Logger.getInstance().createLogfile(logfileName.append("_initial").toString());
        } else if (e.data.gameState == GameControlData.STATE_FINISHED && (lastGameControlData == null || lastGameControlData.gameState != GameControlData.STATE_FINISHED)) {
            Logger.getInstance().createLogfile(logfileName.append("_finished").toString());
        }

        lastGameControlData = e.data;

        // Log the GameController data
        if (e.data != null || changed != 0) {
            Logger.getInstance().log(e.data);
        }

        // send events
        sendEvents(changed);
    }

    /**
     * Updates info about the game when no message was received from the
     * GameController.
     *
     * @param e event
     */
    @Override
    public void gameControlDataTimeout(final GameControlDataTimeoutEvent e) {
        if (LogReplayer.getInstance().isReplaying()) {
            return;
        }

        int changed = 0;

        if (lastGameControlData != null) {
            synchronized (teamNumbers) {
                teamNumbers[TEAM_LEFT] = 0;
                teamNumbers[TEAM_RIGHT] = 0;
                int s = 0;
                for (final Entry<Integer, Collection<RobotState>> entry : robots.entrySet()) {
                    if (!entry.getValue().isEmpty()) {
                        teamNumbers[s++] = entry.getKey();
                        if (s == 2) {
                            break;
                        }
                    }
                }
            }
            changed = CHANGED_LEFT | CHANGED_RIGHT | CHANGED_OTHER;
            Logger.getInstance().createLogfile();
        }
        lastGameControlData = null;

        // send events
        sendEvents(changed);
    }

    /**
     * Handles a message that was received from a robot.
     *
     * @param address IP address of the sender
     * @param teamNumber team number belonging to the port on which the message
     * was received
     * @param message received message
     */
    public void receiveMessage(final String address, final int teamNumber, final SPLTeamMessage message) {
        int changed = 0;

        // update the team info if no GameController info is available
        if (lastGameControlData == null) {
            synchronized (teamNumbers) {
                boolean exists = false;
                for (int i = 0; i < 2; i++) {
                    if (teamNumbers[i] == teamNumber) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) {
                    for (int i = 0; i < 2; i++) {
                        if (teamNumbers[i] == 0) {
                            teamNumbers[i] = teamNumber;

                            // (re)load plugins
                            PluginLoader.getInstance().update(teamNumber);

                            changed |= (i + 1) | CHANGED_OTHER;
                            break;
                        }
                    }
                }
            }
        }


        // create the robot state if it does not yet exist
        RobotState r;
        synchronized (robotsByAddress) {
            r = robotsByAddress.get(address);
            if (r == null) {
                r = new RobotState(address, teamNumber);

                robotsByAddress.put(address, r);
            }

            Collection<RobotState> set = robots.computeIfAbsent(teamNumber, k -> new HashSet<>());
            if (set.add(r)) {
                if (teamNumbers[TEAM_LEFT] == teamNumber) {
                    changed |= CHANGED_LEFT;
                } else if (teamNumbers[TEAM_RIGHT] == teamNumber) {
                    changed |= CHANGED_RIGHT;
                } else {
                    changed |= CHANGED_OTHER;
                }
            }
        }

        // let the robot state handle the message
        r.registerMessage(message);

        // send events
        sendEvents(changed);

        //EB

        ByteBuffer debugPack = ByteBuffer.wrap(message.data).order(ByteOrder.LITTLE_ENDIAN);
        //System.out.println("pos1: " + debugPack.position());
        debugPack.position(12); // skip 8 bytes and header 
        //System.out.println("pos2: " + debugPack.position());
        r.lastGCRDMessage = new GameControlReturnData();

        r.lastGCRDMessage.version  = debugPack.get();// 12
        r.lastGCRDMessage.playerNum = debugPack.get();// 13
        r.lastGCRDMessage.teamNum  = debugPack.get() ;// 14
        r.playerNumber = Integer.valueOf(r.lastGCRDMessage.playerNum);
        r.teamNumber = Integer.valueOf(r.lastGCRDMessage.teamNum);
        r.lastGCRDMessage.fallen = debugPack.get() != 0;// 15
        r.lastGCRDMessage.pose[0] = debugPack.getFloat();// 19
        r.lastGCRDMessage.pose[1] = debugPack.getFloat();// 23
        r.lastGCRDMessage.pose[2] = debugPack.getFloat();// 27
        r.lastGCRDMessage.ballAge = debugPack.getFloat();// 31
        r.lastGCRDMessage.ball[0] = debugPack.getFloat();// 35
        r.lastGCRDMessage.ball[1] = debugPack.getFloat();// 39
        r.lastGCRDMessage.valid = true;
        r.lastGCRDMessage.headerValid = true;
        r.lastGCRDMessage.versionValid = true;
        r.lastGCRDMessage.playerNumValid = true;
        r.lastGCRDMessage.teamNumValid = true;
        r.lastGCRDMessage.fallenValid = true;
        r.lastGCRDMessage.poseValid = true;
        r.lastGCRDMessage.ballValid = true;

        System.out.println("name: "    + address.substring(address.lastIndexOf(".") + 1));
        System.out.println("Version: "    + r.lastGCRDMessage.version);
        System.out.println("PlayerNum: "        + r.playerNumber);
        System.out.println("TeamNum: "          + r.teamNumber);
        System.out.println("fallen: "           + r.lastGCRDMessage.fallen);
        System.out.println("robPosition 0: "    + r.lastGCRDMessage.pose[0]);
        System.out.println("robPosition 1: "    + r.lastGCRDMessage.pose[1]);
        System.out.println("robPosition 2: "    + r.lastGCRDMessage.pose[2]);
        System.out.println("ballAge: "          + r.lastGCRDMessage.ballAge);
        System.out.println("ballPosition 0: "   + r.lastGCRDMessage.ball[0]);
        System.out.println("ballPosition 1: "   + r.lastGCRDMessage.ball[1]);
        // comments may be wrong since I have changed NumOfDataBytes type from uint8_t to uint16_t and now it's placed before role
        System.out.println("NumOfDataBytes: "   + debugPack.getShort()); // 41
        r.role = RobotState.Role.values()[debugPack.get()]; // 42
        System.out.println("role: "             + r.role);

        int CurrentObsNum = debugPack.get(); // current number of obstacles 42
        // you don't need any padding before reading the obstacles
        System.out.println("CurrentObsNum: " + CurrentObsNum);

        // System.out.println("-----------preObs-----------");

        if (CurrentObsNum > Obstacle.MAX_OBS_SIZE) {
            CurrentObsNum = Obstacle.MAX_OBS_SIZE;
        }

        if(CurrentObsNum > 0) {
            // System.out.println("-----------inside-----------");
            // get types, centers, left and right of each obstacle
            int MaxObsNum = Obstacle.MAX_OBS_SIZE;
            // System.out.println("pos1: " + debugPack.position());

            // 43 + 6
            int padding = (((debugPack.position() + MaxObsNum) % 4) == 0) ? 0 : 4 - ((debugPack.position() + MaxObsNum) % 4); // 3
            // System.out.println("pad1: " + padding);
            int[] obsTypes = new int[CurrentObsNum];
            for(int i = 0; i < CurrentObsNum; i++) {
                obsTypes[i] = debugPack.get();
            }
            // 45 ( Current = 2 )
            // System.out.println("pos2: " + debugPack.position());
            debugPack.position(debugPack.position() + MaxObsNum - CurrentObsNum + padding); //jump over the empty position and padding (if you have any)
            // 52
            // System.out.println("pos3: " + debugPack.position());
            padding = (((debugPack.position() + 2*4*MaxObsNum) % 4) == 0) ? 0 : 4 - ((debugPack.position() + 2*4*MaxObsNum) % 4);// 0
            // System.out.println("pad2: " + padding);
            ///////// giusto

            float[][] obsCenters = new float[CurrentObsNum][2];
            for(int j = 0; j < 2*CurrentObsNum; j++) {
                // System.out.println("j: " + j);
                if (j % 2 == 0 ) {
                    obsCenters[j/2][0] = debugPack.getFloat(); // x
                    //System.out.println("X: " + obsCenters[j/2][0]);
                }
                else {
                    obsCenters[j/2][1] = debugPack.getFloat(); // y
                    //System.out.println("Y: " + obsCenters[j/2][1]);
                    //System.out.println("pos8: " + debugPack.position());
                }
            }
            // 56 + 2*4*2 + 0 = 68
            // System.out.println("pos4: " + debugPack.position());
            debugPack.position(debugPack.position() + 8*(MaxObsNum - CurrentObsNum) + padding); //jump over the empty position and padding (if you have any)
            // 68 + 32 + 0 = 100
            // System.out.println("pos5: " + debugPack.position());

            int[] obsLastSeen = new int[CurrentObsNum];
            for(int j = 0; j < CurrentObsNum; j++) {
                obsLastSeen[j] = debugPack.getInt();
            }
            // 100 + 4*2 = 108
            // System.out.println("pos6: " + debugPack.position());
            r.seenObstacles.clear();
            // 108 + 4*6 % 4 = 0
            padding = (((debugPack.position() + 4*MaxObsNum) % 4) == 0) ? 0 : 4 - ((debugPack.position() + 4*MaxObsNum) % 4);
            // System.out.println("pad3: " + padding);
            for(int i = 0; i<CurrentObsNum; i++) {
                Obstacle obs = new Obstacle(obsTypes[i], obsCenters[i], obsLastSeen[i]);
                System.out.println(obs.toString());
                r.seenObstacles.add(obs); // add to list of seen obstacles
            }
            // 108 + 4*4 + 0 = 124
            debugPack.position(debugPack.position() + 4*(MaxObsNum - CurrentObsNum) + padding); //jump over the empty position and padding (if you have any)
            
        }
        else {
            // if you change something on the structure of the message then you will have to change this accordingly!
            debugPack.position(debugPack.position() + Obstacle.MAX_OBS_SIZE*(1 + 4 + 8)); // Types(1B), Centers(4B), lastSeen(4B)
        }

        short messageBudget = debugPack.getShort();
        System.out.println("Budget: "+messageBudget);
        //System.out.println("-----------secs-----------");
        short secsRemaining = debugPack.getShort();
        System.out.println("SecsRemaining: "+secsRemaining);
        //System.out.println("pos8: " + debugPack.position()); // 126

        // extract Arm Contact
        boolean armContact = ((debugPack.get() == 1) || (debugPack.get() == 1));// 128
        //System.out.println("pos9: " + debugPack.position());
        int armPush1 = debugPack.get();// 129
        //System.out.println("pos10: " + debugPack.position());
        int armPush2 = debugPack.get();// 130 
        //System.out.println("pos11: " + debugPack.position());

        int lastArmContact1 = debugPack.getInt(); // 134
        //System.out.println("pos12: " + debugPack.position());
        int lastArmContact2 = debugPack.getInt(); // 138
        //System.out.println("pos13: " + debugPack.position());

        if(lastArmContact1 < lastArmContact2) {
            r.currentContact = new Contact(armContact, armPush1, lastArmContact1);
        }
        else {
            r.currentContact = new Contact(armContact, armPush2, lastArmContact2);
        }
        // System.out.println("-----------arm-----------");
        System.out.println(r.currentContact.toString());

        r.lastWhistleDetection = debugPack.getInt();

        r.whistleDetected = RobotState.WhistleDetector.values()[debugPack.get()];

        System.out.println("Whistle detected: "+r.whistleDetected+ " since: "+r.lastWhistleDetection);

        debugPack.position(debugPack.position() + 3);

        r.TeamBallPos[0] = debugPack.getFloat();
        r.TeamBallPos[1] = debugPack.getFloat();

        r.TeamBallVel[0] = debugPack.getFloat();
        r.TeamBallVel[1] = debugPack.getFloat();

        System.out.println("TeamBallPos: "+r.TeamBallPos[0]+" , "+r.TeamBallPos[1]);
        System.out.println("TeamBallVel: "+r.TeamBallVel[0]+" , "+r.TeamBallVel[1]);

        if (Float.isNaN(r.TeamBallPos[0]) || Float.isNaN(r.TeamBallPos[1]) || Float.isNaN(r.TeamBallVel[0]) || Float.isNaN(r.TeamBallVel[1])) {
            // Ready position problem
            r.TeamBallPos[0] = 0;
            r.TeamBallPos[1] = 0;
            r.TeamBallVel[0] = 0;
            r.TeamBallVel[1] = 0;
        }

        r.batteryLevel = debugPack.getFloat() * 100; //  %
        r.cpuTemperature = debugPack.getFloat(); // °C

        System.out.println("Battery Level: "+r.batteryLevel+" , Temperature: "+r.cpuTemperature);

        r.refereePose = debugPack.get() != 0;

        System.out.println("Referee Pose: "+r.refereePose);


        System.out.println("-----------------------");
        //EB
    }

    /**
     * Handles a GameController return message that was received from a robot.
     *
     * @param address IP address of the sender
     * @param message received message
     */
    public void receiveMessage(final String address, final GameControlReturnData message) {
        // only handle if there is an active GameController
        if (lastGameControlData == null) {
            return;
        }

        int changed = 0;
        RobotState r;
        synchronized (robotsByAddress) {
            // only handle if the player belongs to one of the playing teams
            if (message.teamNum != teamNumbers[TEAM_LEFT] && message.teamNum != teamNumbers[TEAM_RIGHT]) {
                return;
            }

            r = robotsByAddress.get(address);
            if (r == null) {
                r = new RobotState(address, message.teamNum);

                robotsByAddress.put(address, r);
            }

            Collection<RobotState> set = robots.computeIfAbsent((int) message.teamNum, k -> new HashSet<>());
            if (set.add(r)) {
                if (teamNumbers[TEAM_LEFT] == message.teamNum) {
                    changed |= CHANGED_LEFT;
                } else if (teamNumbers[TEAM_RIGHT] == message.teamNum) {
                    changed |= CHANGED_RIGHT;
                }
            }
        }

        // let the robot state handle the message
        r.registerMessage(message);

        // send events
        sendEvents(changed);
    }

    private void sendEvents(final int changed) {
        boolean leftSent = false;
        boolean rightSent = false;

        if ((changed & CHANGED_OTHER) != 0) {
            // Use Lists instead of Sets so that multiple robots with the same player number appear in the TCM
            // https://github.com/bhuman/GameController/pull/37
            final List<RobotState> rs = new ArrayList<>();
            synchronized (robotsByAddress) {
                for (final Entry<Integer, Collection<RobotState>> entry : robots.entrySet()) {
                    if (entry.getKey() == teamNumbers[TEAM_LEFT]) {
                        if ((changed & CHANGED_LEFT) != 0) {
                            final List<RobotState> list = new ArrayList<>(entry.getValue());
                            list.sort(playerNumberComparator);
                            fireEvent(new TeamEvent(this, outputSide(TEAM_LEFT), teamNumbers[TEAM_LEFT], list));
                            leftSent = true;
                        }
                    } else if (entry.getKey() == teamNumbers[TEAM_RIGHT]) {
                        if ((changed & CHANGED_RIGHT) != 0) {
                            final List<RobotState> list = new ArrayList<>(entry.getValue());
                            list.sort(playerNumberComparator);
                            fireEvent(new TeamEvent(this, outputSide(TEAM_RIGHT), teamNumbers[TEAM_RIGHT], list));
                            rightSent = true;
                        }
                    } else {
                        rs.addAll(entry.getValue());
                    }
                }
            }
            rs.sort(playerNumberComparator);
            fireEvent(new TeamEvent(this, TEAM_OTHER, 0, rs));
        }

        if (!leftSent && (changed & CHANGED_LEFT) != 0) {
            final Collection<RobotState> rs;
            synchronized (robotsByAddress) {
                rs = robots.get(teamNumbers[TEAM_LEFT]);
            }
            final List<RobotState> list = new ArrayList<>();
            if (rs != null) {
                list.addAll(rs);
                list.sort(playerNumberComparator);
            }
            fireEvent(new TeamEvent(this, outputSide(TEAM_LEFT), teamNumbers[TEAM_LEFT], list));
        }

        if (!rightSent && (changed & CHANGED_RIGHT) != 0) {
            final Collection<RobotState> rs;
            synchronized (robotsByAddress) {
                rs = robots.get(teamNumbers[TEAM_RIGHT]);
            }
            final List<RobotState> list = new ArrayList<>();
            if (rs != null) {
                list.addAll(rs);
                list.sort(playerNumberComparator);
            }
            fireEvent(new TeamEvent(this, outputSide(TEAM_RIGHT), teamNumbers[TEAM_RIGHT], list));
        }
    }

    private void fireEvent(final TeamEvent e) {
        for (final TeamEventListener listener : listeners.getListeners(TeamEventListener.class)) {
            listener.teamChanged(e);
        }
    }

    /**
     * Returns the team color of the given team. The team color is either sent
     * by the game controller or given by the GameController configuration.
     *
     * @param teamNumber number of the team
     * @param playerNumber number of the player
     * @return the team color
     * @see TeamInfo#fieldPlayerColor and TeamInfo#goalkeeperColor
     */
    public int getTeamColor(final int teamNumber, final int playerNumber) {
        if (lastGameControlData == null || (lastGameControlData.team[0].teamNumber != teamNumber && lastGameControlData.team[1].teamNumber != teamNumber)) {
            String[] colorStrings = null;
            try {
                colorStrings = Teams.getColors(teamNumber);
            } catch (final NullPointerException | ArrayIndexOutOfBoundsException e) {
            }
            if (colorStrings == null || colorStrings.length < 1) {
                if (teamNumber == teamNumbers[TEAM_RIGHT]) {
                    return GameControlData.TEAM_RED;
                } else {
                    return GameControlData.TEAM_BLUE;
                }
            } else if (colorStrings[0].equals("blue")) {
                return GameControlData.TEAM_BLUE;
            } else if (colorStrings[0].equals("red")) {
                return GameControlData.TEAM_RED;
            } else if (colorStrings[0].equals("yellow")) {
                return GameControlData.TEAM_YELLOW;
            } else if (colorStrings[0].equals("black")) {
                return GameControlData.TEAM_BLACK;
            } else if (colorStrings[0].equals("green")) {
                return GameControlData.TEAM_GREEN;
            } else if (colorStrings[0].equals("orange")) {
                return GameControlData.TEAM_ORANGE;
            } else if (colorStrings[0].equals("purple")) {
                return GameControlData.TEAM_PURPLE;
            } else if (colorStrings[0].equals("brown")) {
                return GameControlData.TEAM_BROWN;
            } else if (colorStrings[0].equals("gray")) {
                return GameControlData.TEAM_GRAY;
            } else {
                return GameControlData.TEAM_WHITE;
            }
        }
        final int index = lastGameControlData.team[0].teamNumber == teamNumber ? 0 : 1;
        return lastGameControlData.team[index].goalkeeper == playerNumber ? lastGameControlData.team[index].goalkeeperColor : lastGameControlData.team[index].fieldPlayerColor;
    }

    /**
     * Returns the most recently received GameControlData.
     *
     * @return GameControlData of null if none was received recently
     */
    public GameControlData getLastGameControlData() {
        return lastGameControlData;
    }

    /**
     * Returns the team name of the given team.
     *
     * @param teamNumber number of the team
     * @param withNumber whether the team number should be in the returned
     * string
     * @param withPrefix whether the pre- or suffix "Team" should be included
     * @return the team name
     */
    public String getTeamName(final Integer teamNumber, final boolean withNumber, final boolean withPrefix) {
        final String[] teamNames;
        try {
            teamNames = Teams.getNames(withNumber);
        } catch (final NullPointerException | ArrayIndexOutOfBoundsException e) {
            return null;
        }
        if (teamNumber != null) {
            if (teamNumber < teamNames.length && teamNames[teamNumber] != null) {
                return ((withPrefix ? "Team " : "") + teamNames[teamNumber]);
            } else {
                return ("Unknown" + (withPrefix ? " Team" : "") + (withNumber ? " (" + teamNumber + ")" : ""));
            }
        } else {
            return "Unknown" + (withPrefix ? " Team" : "");
        }
    }

    /**
     * Returns whether the team sides are mirrored.
     *
     * @return boolean
     */
    public boolean isMirrored() {
        return mirrored;
    }

    /**
     * Sets whether the team sides are mirrored.
     *
     * @param mirrored boolean
     */
    public void setMirrored(final boolean mirrored) {
        if (mirrored != this.mirrored) {
            this.mirrored = mirrored;
            sendEvents(CHANGED_LEFT | CHANGED_RIGHT);
        }
    }

    private int outputSide(final int side) {
        return mirrored ? (side == 0 ? 1 : (side == 1 ? 0 : side)) : side;
    }

    /**
     * Registers a GUI component as a listener receiving events about team
     * changes.
     *
     * @param listener component
     */
    public void addListener(final TeamEventListener listener) {
        listeners.add(TeamEventListener.class, listener);
        sendEvents(CHANGED_LEFT | CHANGED_RIGHT | CHANGED_OTHER);
    }

    /**
     * Unregisters a GUI component as a listener receiving events about team
     * changes.
     *
     * @param listener component
     */
    public void removeListener(final TeamEventListener listener) {
        listeners.remove(TeamEventListener.class, listener);
    }

}
