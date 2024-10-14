package teamcomm.data;

import data.PlayerInfo;
import data.GameControlReturnData;
import data.SPLTeamMessage;
import java.util.LinkedList;
import java.util.ListIterator;
import javax.swing.event.EventListenerList;
import teamcomm.data.event.RobotStateEvent;
import teamcomm.data.event.RobotStateEventListener;
import teamcomm.data.Obstacle;
import teamcomm.data.Contact;

import java.util.Arrays;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Class representing the state of a robot.
 *
 * @author Felix Thielke
 */
public class RobotState {

    public enum ConnectionStatus {

        INACTIVE(10000),
        OFFLINE(5000),
        HIGH_LATENCY(2000),
        ONLINE(0);

        public final int threshold;

        ConnectionStatus(final int threshold) {
            this.threshold = threshold;
        }
    }

    public enum Role {

        none,
        goalkeeper,
        ballplayer,
        undefined,

        goalie,
        striker,
        defenderone,
        defendertwo,
        supporter,
        jolly,
        libero,

        penaltyStriker,
        penaltyKeeper,
        planStriker,
        planJolly,
        activeSearcher,
        passiveSearcher

    }

    public enum Name {
        None,
        Augusto, // 16
        Caligola, // 17
        Cesare, // 18
        Claudio, // 19
        Nerone, // 20
        Tiberio, // 21
        Ethan, // 22
        Erika, // 23
        Adriano, // 24
        Commodo, // 25
        Domiziano, // 26
        MarcoAurelio, // 27
        Tito, // 28
        Traiano, // 29
        Vespasiano, // 30
        Int, // 31
    }

    public enum WhistleDetector {
        dontKnow,
        notDetected,
        isDetected,
    }

    private static final int AVERAGE_CALCULATION_TIME = 10000;

    private final String address;
    private SPLTeamMessage lastTeamMessage;
    public GameControlReturnData lastGCRDMessage; // private
    private long lastTeamMessageTimestamp;
    private long lastGCRDMessageTimestamp;
    private final LinkedList<Long> recentTeamMessageTimestamps = new LinkedList<>();
    private final LinkedList<Long> recentGCRDMessageTimestamps = new LinkedList<>();
    private int teamMessageCount = 0;
    private int gcrdMessageCount = 0;
    private int illegalTeamMessageCount = 0;
    private int illegalGCRDMessageCount = 0;
    public int teamNumber; // private final
    public Integer playerNumber = null; // private 
    private byte penalty = PlayerInfo.PENALTY_NONE;
    private ConnectionStatus lastConnectionStatus = ConnectionStatus.ONLINE;
    public Role role = Role.none;
    public Name name = Name.None;
    public LinkedList<Obstacle> seenObstacles = new LinkedList<>();
    public Contact currentContact;
    public WhistleDetector whistleDetected = WhistleDetector.dontKnow;
    public int lastWhistleDetection;

    public float[] TeamBallPos = new float[2];
    public float[] TeamBallVel = new float[2];

    public float batteryLevel;
    public float cpuTemperature;

    public boolean refereePose;

    private final EventListenerList listeners = new EventListenerList();

    /**
     * Constructor.
     *
     * @param address IP address of the robot
     * @param teamNumber team number associated with the port the robot sends
     * his messages on
     */
    public RobotState(final String address, final int teamNumber) {
        this.address = address;
        this.teamNumber = teamNumber;

        int index = Integer.parseInt(address.substring(address.lastIndexOf(".") + 1)) - 15;
        if(index < 0)
            index = 0;

        this.name = Name.values()[index];
    }

    /**
     * Handles a SPL team message received by the robot this object corresponds to.
     *
     * @param message received SPL team message or null if the message was invalid
     */
    public void registerMessage(final SPLTeamMessage message) {
        if (!message.valid) {
            illegalTeamMessageCount++;
        }
        lastTeamMessage = message;
        lastTeamMessageTimestamp = System.currentTimeMillis();
        synchronized (recentTeamMessageTimestamps) {
            recentTeamMessageTimestamps.addFirst(lastTeamMessageTimestamp);
        }
        teamMessageCount++;

        for (final RobotStateEventListener listener : listeners.getListeners(RobotStateEventListener.class)) {
            listener.robotStateChanged(new RobotStateEvent(this));
            listener.connectionStatusChanged(new RobotStateEvent(this));
        }
    }

    /**
     * Handles a GameController return message received by the robot this object corresponds to.
     *
     * @param message received GameController return message
     */
    public void registerMessage(final GameControlReturnData message) {
        if (!message.valid) {
            illegalGCRDMessageCount++;
        }
        if (message.playerNumValid) {
            playerNumber = (int) message.playerNum;
        }
        lastGCRDMessage = message;
        lastGCRDMessageTimestamp = System.currentTimeMillis();
        synchronized (recentGCRDMessageTimestamps) {
            recentGCRDMessageTimestamps.addFirst(lastGCRDMessageTimestamp);
        }
        gcrdMessageCount++;

        for (final RobotStateEventListener listener : listeners.getListeners(RobotStateEventListener.class)) {
            listener.robotStateChanged(new RobotStateEvent(this));
            listener.connectionStatusChanged(new RobotStateEvent(this));
        }
    }

    /**
     * Returns the IP address of the robot.
     *
     * @return IP address
     */
    public String getAddress() {
        return address;
    }

    /**
     * Returns the most recent legal team message received from this robot.
     *
     * @return message
     */
    public SPLTeamMessage getLastTeamMessage() {
        return lastTeamMessage;
    }

    /**
     * Returns the most recent legal GameController return message received from this robot.
     *
     * @return message
     */
    public GameControlReturnData getLastGCRDMessage() {
        return lastGCRDMessage;
    }

    /**
     * Returns the average number of team messages per second.
     *
     * @return number of messages per second
     */
    public double getTeamMessagesPerSecond() {
        synchronized (recentTeamMessageTimestamps) {
            final ListIterator<Long> it = recentTeamMessageTimestamps.listIterator(recentTeamMessageTimestamps.size());

            while (it.hasPrevious() && lastTeamMessageTimestamp - it.previous() > AVERAGE_CALCULATION_TIME) {
                it.remove();
            }

            return !recentTeamMessageTimestamps.isEmpty() ? ((recentTeamMessageTimestamps.size() - 1) * 1000.0 / Math.max(1000, lastTeamMessageTimestamp - recentTeamMessageTimestamps.getLast())) : 0;
        }
    }

    /**
     * Returns the average number of GameController return messages per second.
     *
     * @return number of messages per second
     */
    public double getGCRDMessagesPerSecond() {
        synchronized (recentGCRDMessageTimestamps) {
            final ListIterator<Long> it = recentGCRDMessageTimestamps.listIterator(recentGCRDMessageTimestamps.size());

            while (it.hasPrevious() && lastGCRDMessageTimestamp - it.previous() > AVERAGE_CALCULATION_TIME) {
                it.remove();
            }

            return !recentGCRDMessageTimestamps.isEmpty() ? ((recentGCRDMessageTimestamps.size() - 1) * 1000.0 / Math.max(1000, lastGCRDMessageTimestamp - recentGCRDMessageTimestamps.getLast())) : 0;
        }
    }

    /**
     * Updates the current network status of the robot internally. Sends events
     * about a change of the connection status if needed.
     *
     * @return the current connection status
     */
    public ConnectionStatus updateConnectionStatus() {
        final ConnectionStatus c = getConnectionStatus();
        if (c != lastConnectionStatus) {
            lastConnectionStatus = c;
            for (final RobotStateEventListener listener : listeners.getListeners(RobotStateEventListener.class)) {
                listener.connectionStatusChanged(new RobotStateEvent(this));
            }
        }
        return c;
    }

    /**
     * Returns the current network status of the robot.
     *
     * @return connection status
     */
    public ConnectionStatus getConnectionStatus() {
        final long timeSinceLastMessage = System.currentTimeMillis() - Math.max(lastTeamMessageTimestamp, lastGCRDMessageTimestamp);
        for (final ConnectionStatus c : ConnectionStatus.values()) {
            if (timeSinceLastMessage >= c.threshold) {
                return c;
            }
        }

        return ConnectionStatus.ONLINE;
    }

    /**
     * Returns the total count of received team messages.
     *
     * @return total message count
     */
    public int getTeamMessageCount() {
        return teamMessageCount;
    }

    /**
     * Returns the total count of illegal team messages.
     *
     * @return illegal message count
     */
    public int getIllegalTeamMessageCount() {
        return illegalTeamMessageCount;
    }

    /**
     * Returns the time when the last team message was received.
     *
     * @return The time in ms.
     */
    public long getLastTeamMessageTimestamp() {
        return lastTeamMessageTimestamp;
    }

    /**
     * Returns the ratio of illegal team messages to the total count of team messages.
     *
     * @return ratio
     */
    public double getIllegalTeamMessageRatio() {
        return (double) illegalTeamMessageCount / (double) teamMessageCount;
    }

    /**
     * Returns the total count of received GameController return messages.
     *
     * @return total message count
     */
    public int getGCRDMessageCount() {
        return gcrdMessageCount;
    }

    /**
     * Returns the total count of illegal GameController return messages.
     *
     * @return illegal message count
     */
    public int getIllegalGCRDMessageCount() {
        return illegalGCRDMessageCount;
    }

    /**
     * Returns the ratio of illegal GameController return messages to the total count of GameController return messages.
     *
     * @return ratio
     */
    public double getIllegalGCRDMessageRatio() {
        return (double) illegalGCRDMessageCount / (double) gcrdMessageCount;
    }

    /**
     * Returns the team number of this robot.
     *
     * @return team number
     */
    public int getTeamNumber() {
        return teamNumber;
    }

    /**
     * Returns the player number of the robot or null if it did not send any.
     *
     * @return player number or null
     */
    public Integer getPlayerNumber() {
        return playerNumber;
    }

    /**
     * Returns the current penalty of the robot.
     *
     * @return penalty
     * @see PlayerInfo#penalty
     */
    public byte getPenalty() {
        return penalty;
    }

    /**
     * Sets the current penalty of the robot.
     *
     * @param penalty penalty
     * @see PlayerInfo#penalty
     */
    public void setPenalty(final byte penalty) {
        this.penalty = penalty;
    }

    /**
     * Registers a GUI component as a listener receiving events when this robot
     * sends a message.
     *
     * @param listener component
     */
    public void addListener(final RobotStateEventListener listener) {
        listeners.add(RobotStateEventListener.class, listener);
    }

    /**
     * Removes an event listener from this robot.
     *
     * @param listener listener
     */
    public void removeListener(final RobotStateEventListener listener) {
        listeners.remove(RobotStateEventListener.class, listener);
    }
}
