from rclpy.node import Node
from rostron_ia_ms.utils.world import World
from rostron_utils.rostron_utils.angle_radian import AngleRadian
from math import sin, cos, pi, sqrt, atan, acos, asin

# from rostron_interfaces.action import MoveTo

from rostron_navigation.rostron_navigation.primitive.move_to import MoveTo
from rclpy.action.server import ServerGoalHandle

class Striker(Node):
    """
    A améliorer :
    * Gérer la puissance du kick en fonction de la distance demandée
    * Gerer le cas lorsqu'il faut tirer dans le sens opposé strict de la balle
      (il faut éviter la balle)
    * Bug lorsqu'on passe du moveTo au kick : le dribbler (spin power de Hardware) s'arrete 
      et decolle la balle du robot => empeche le tir car balle décollé (probleme en simu)
    """

    def __init__(self, id):
        super().__init__("striker")
        # self.create_timer(0.16, self.update)
        self.id = id
        self.ball_position = (0,0)
        self.distance_ball = 0.2
        World().init(self)
    
    def update(self):
        pass
        
    def update_pose_ball(self):
        """Update the values ​​of the ball position"""
        self.ball_position = (World().ball.position.x, World().ball.position.y)

    def first_pose(self,pose,theta):
        """ Returns a pose behind a position and aligned with the angle theta"""
        return (pose[0]+self.distance_ball*cos(theta-pi), pose[1]+self.distance_ball*sin(theta-pi))

    def goal_pose(self,pose,theta):
        """ Returns a pose close to the point and aligned with the angle theta"""
        return (pose[0]+self.distance_ball*cos(theta), pose[1]+self.distance_ball*sin(theta))
    
    # Formule de Distance #
    def distance(self, pose1, pose2):
        """Returns the distance between two objects"""
        return sqrt( (pose2[0] - pose1[0])**2 + (pose2[1] - pose1[1])**2 )
    
    def distanceToBall(self, pose):
        """Returns the distance between one robot and the ball"""
        return sqrt( (World().ball.position.x - pose[0])**2 + (World().ball.position.y - pose[1])**2 )
    
    def distanceGoalToBall(self):
        """Returns the distance between the center of the goal and the ball"""
        return sqrt( (World().ball.position.x + 4.6)**2 + (World().ball.position.y - 0)**2 )
    ##########################################################################################
    
    # Formule de Trigonométrie #
    def angleThetaCatchBall(self, distanceHypotenuse, distanceAdjacent):
        return acos(distanceAdjacent/distanceHypotenuse)
    
    def angleAlphaRotationGoal(self, distanceBalltoGoal):
        return atan(0.56/distanceBalltoGoal)
    ##########################################################################################

    def move_to_ball_and_kick(self, goal_handle: ServerGoalHandle):
        self.update_pose_ball()

        # Important points #
        centreGoal =  (-4.6, 0.0)
        thisRobot = World().allies[self.id]
        centreDribbler = (thisRobot.pose.position.x, thisRobot.pose.position.y)     
        ############################
        
        # Initial params # 
        pose = (-4.6, 0.0)      # Est ce que ça marche, ne pas mettre les coordonnées du robot en question plutôt ?
        kick_type = 1
        ##################

        robot = MoveTo(self.id)
        ball_position = (World().ball.position.x, World().ball.position.y)
        vector_angle = (pose[0]-ball_position[0], pose[1]-ball_position[1])
        vector_axis = (1,0) # right vector
        theta = AngleRadian.angle_between(vector_axis, vector_angle)

        robot.move_to(self.first_pose(ball_position,theta),theta)
        self.distance_ball = 0.1
        robot.move_by([self.first_pose(ball_position, theta),self.ball_position, self.goal_pose(ball_position, theta)],theta, True) 
        
        # Second step : rotate with the ball to the goal #
        distanceBallToGoal = self.distanceGoalToBall()
        alpha = pi/17
        robot.move_by([self.goal_pose(ball_position, theta)],alpha, True) 

        # Move to pose after the ball without dribbling to keep the ball close to kicker
        robot.move_by([self.goal_pose(ball_position, theta)],alpha) # disable the dribbler before kicking the ball

        # Third step : kick the ball #
        robot.kick(kick_type)

        goal_handle.succeed()
        result = MoveTo.Result()
        return result

# def move_to_ball_and_kick_val(self, goal_handle: ServerGoalHandle):
#         self.update_pose_ball()

#         # Important points #
#         centreGoal =  (-4.6, 0.0)
#         thisRobot = World().allies[self.id]
#         centreDribbler = (thisRobot.pose.position.x, thisRobot.pose.position.y)   
#         ############################
        
#         # Initial params # 
#         pose = (-4.6, 0.0)      # Est ce que ça marche, ne pas mettre les coordonnées du robot en question plutôt ?
#         kick_type = 1
#         ##################

#         # First step : catch the ball in the dribbler #
#         robot = MoveToStrategie(self.id)
#         ball_position = (World().ball.position.x, World().ball.position.y)
#         vector_angle = (pose[0]-ball_position[0], pose[1]-ball_position[1])
#         vector_axis = (1,0) # right vector
#         theta = AngleRadian.angle_between(vector_axis, vector_angle)
#         robot.move_to(self.first_pose(ball_position,theta),theta)

        # Second step : rotate with the ball to the goal #
        # distanceBallToGoal = self.distanceGoalToBall()
        # alpha = pi/17

        # self.distance_ball = 0.1
        # robot.move_by([self.first_pose(ball_position, theta),self.ball_position, self.goal_pose(ball_position, theta)],alpha, True) 
        
#         # Move to pose after the ball without dribbling to keep the ball close to kicker
#         robot.move_by([self.goal_pose(ball_position, theta)],alpha) # disable the dribbler before kicking the ball

#         # Third step : kick the ball #
#         robot.kick(kick_type)

#         goal_handle.succeed()
#         result = MoveTo.Result()
#         return result
