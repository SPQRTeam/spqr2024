package teamcomm.gui.drawings.common;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.glu.GLUquadric;
import data.GameControlData;
import data.GameControlReturnData;
import data.PlayerInfo;
import data.Rules;
import data.SPL;
import teamcomm.data.GameState;
import teamcomm.data.RobotState;
import teamcomm.data.Obstacle;
import teamcomm.gui.Camera;
import teamcomm.gui.drawings.PerPlayer;
import teamcomm.gui.drawings.RoSi2Loader;
import teamcomm.gui.drawings.Text;

/**
 * Drawing for a robot.
 *
 * @author Felix Thielke
 */
public class Player extends PerPlayer {

    private static final float CYLINDER_HEIGHT = 0.5f;
    private static final float CYLINDER_RADIUS = 0.0625f;
    private static final float DISK_RADIUS = 0.1025f;

    private static String getModelName(final int color) {
        switch (color) {
            case GameControlData.TEAM_BLUE:
                return "robotBlue";
            case GameControlData.TEAM_RED:
                return "robotRed";
            case GameControlData.TEAM_BLACK:
                return "robotBlack";
            case GameControlData.TEAM_YELLOW:
                return "robotYellow";
            case GameControlData.TEAM_WHITE:
                return "robotWhite";
            case GameControlData.TEAM_GREEN:
                return "robotGreen";
            case GameControlData.TEAM_ORANGE:
                return "robotOrange";
            case GameControlData.TEAM_PURPLE:
                return "robotPurple";
            case GameControlData.TEAM_BROWN:
                return "robotBrown";
            case GameControlData.TEAM_GRAY:
                return "robotGray";
        }

        return "robotWhite";
    }

    @Override
    protected void init(GL2 gl) {
        RoSi2Loader.getInstance().cacheModels(gl, new String[]{"robotBlue", "robotRed", "robotBlack", "robotYellow", "robotWhite", "robotGreen", "robotOrange", "robotPurple", "robotBrown", "robotGray"});
    }

    @Override
    public void draw(final GL2 gl, final RobotState player, final Camera camera) {
        final GameControlReturnData msg = player.getLastGCRDMessage();

        // team ball
        gl.glTranslatef(player.TeamBallPos[0] / 1000.f, player.TeamBallPos[1] / 1000.f, 0);
        gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleBlack"));
        gl.glTranslatef(-player.TeamBallPos[0] / 1000.f, -player.TeamBallPos[1] / 1000.f, 0);

        if (msg != null && msg.poseValid) {
            gl.glPushMatrix();

            if (player.getPenalty() != PlayerInfo.PENALTY_NONE && !(Rules.league instanceof SPL && player.getPenalty() == PlayerInfo.PENALTY_SPL_ILLEGAL_MOTION_IN_SET)) {
                gl.glTranslatef(-msg.playerNum, -3.5f, 0);
                gl.glRotatef(-90, 0, 0, 1);
            } else {
                gl.glTranslatef(msg.pose[0] / 1000.f, msg.pose[1] / 1000.f, 0);
                gl.glRotatef((float) Math.toDegrees(msg.pose[2]), 0, 0, 1);

                if (msg.fallenValid && msg.fallen) {
                    gl.glTranslatef(0, 0, 0.05f);
                    gl.glRotatef(90, 0, 1, 0);
                }
            }

            gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, getModelName(GameState.getInstance().getTeamColor(player.getTeamNumber(), player.getPlayerNumber()))));
            

            // obstacles
            if (player.seenObstacles.size() > 0) {
                    for (Obstacle obs: player.seenObstacles) {
                        gl.glTranslatef(obs.center[0] / 1000.f, obs.center[1] / 1000.f, 0); // translate to obstacle
                        final GLU glu = GLU.createGLU(gl);
                        final GLUquadric q = glu.gluNewQuadric();
                        switch (obs.type) {
                            case goalpost:
                                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleOrange"));
                                break;
                                
                            case unknown:
                                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleBlack"));
                                break;
                            case someRobot:
                                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstaclePurple"));
                                glu.gluCylinder(q, CYLINDER_RADIUS, CYLINDER_RADIUS, 0.5, 16, 1);
                                break;
                            case fallenSomeRobot:
                                gl.glColorMaterial(GL2.GL_FRONT, GL2.GL_AMBIENT_AND_DIFFUSE);
                                gl.glColor4f(0.5f, 0, 0.5f, 1f);
                                gl.glEnable(GL2.GL_BLEND);
                                gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
                                glu.gluDisk(q, 0, DISK_RADIUS, 16, 10);
                                break;

                            case opponent:
                                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleBlue"));
                                glu.gluCylinder(q, CYLINDER_RADIUS, CYLINDER_RADIUS, 0.5, 16, 1);
                                break;
                            case fallenOpponent:
                                gl.glColorMaterial(GL2.GL_FRONT, GL2.GL_AMBIENT_AND_DIFFUSE);
                                gl.glColor4f(0, 0, 1f, 1f);
                                gl.glEnable(GL2.GL_BLEND);
                                gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
                                glu.gluDisk(q, 0, DISK_RADIUS, 16, 10);
                                break;

                            case teammate:
                                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleRed"));
                                glu.gluCylinder(q, CYLINDER_RADIUS, CYLINDER_RADIUS, 0.5, 16, 1);
                                break;
                            case fallenTeammate:
                                gl.glColorMaterial(GL2.GL_FRONT, GL2.GL_AMBIENT_AND_DIFFUSE);
                                gl.glColor4f(1f, 0, 0, 1f);
                                gl.glEnable(GL2.GL_BLEND);
                                gl.glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
                                glu.gluDisk(q, 0, DISK_RADIUS, 16, 10);
                                break;
                        }
                        glu.gluDeleteQuadric(q);
                        gl.glDisable(GL2.GL_BLEND);
                        gl.glTranslatef(-obs.center[0] / 1000.f, -obs.center[1] / 1000.f, 0); // translate back to robot
                    }
            }

            gl.glTranslatef(0, 0, 0.9f); // translate above the number

            // whistle
            if (player.whistleDetected == RobotState.WhistleDetector.isDetected) {
                camera.turnTowardsCamera(gl);
                //Text.drawText("W", 0, 0, 0.35f, Rules.league.teamColor[GameState.getInstance().getTeamColor(player.getTeamNumber(), player.getPlayerNumber())].getRGBColorComponents(null));
                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstacleYellow"));
            }
            // referee
            if (player.refereePose) {    
                camera.turnTowardsCamera(gl);
                gl.glCallList(RoSi2Loader.getInstance().loadModel(gl, "obstaclePink"));
            }
            
            gl.glPopMatrix();
        }
    }

    @Override
    public boolean hasAlpha() {
        return false;
    }

    @Override
    public int getPriority() {
        return 0;
    }
}
