import rclpy                            #import of essentials libs
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan   
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.lin_vel = 0                        #declaration of variables
        self.rot_vel = 0
        self.sum_x = 0
        self.sum_y = 0
        self.x_centre = 0 
        self.publisher= self.create_publisher(Twist,'cmd_vel' , 10)     #TODO & allows the publication
        self.subscription = self.create_subscription(                   #allows data to be received from the lidar.
            LaserScan,
            'scan',
            self.listener_callback,
            10)
            #qos_profile_sensor_data)                                    
        self.subscription  # prevent unused variable warning
        #self.get_logger().info ("debut init")                          #Step debug 1

    def listener_callback(self, msg):
        #self.get_logger().info ("debut ecoute")                        #Step debug 2
        #self.get_logger().info ('I heard: "%s"' % msg)                    #print some info for debug session
        #self.get_logger().info ('Distances : ' "%s" %msg.ranges)
        #self.get_logger().info ('Range max : ' "%s" %msg.ranges)
        #self.get_logger().info ('angle min : ' "%s" %msg.angle_min)
        #self.get_logger().info ('angle max : ' "%s" %msg.angle_max)    
        #self.get_logger().info ('temps entre 2 scans : ' "%s" %msg.scan_time)  
        #self.get_logger().info ('time_increment  : ' "%s" %msg.time_increment)  
       
       # Loop variable
        i = -1
        tour =0
        
        # Variable to be modified for the perception area of the robot
        r_min = 1
        r_max =3
        angle_max = 300
        angle_min = 60
        factor_lin = 0
        factor_rot = 1
        
        # Creation of a twist object
        msg_twist =Twist()

        # Perception area
        self.sum_x = 0
        self.sum_y =0
        
        for r in msg.ranges :
            i=i+1 
            theta = msg.angle_min +msg.angle_increment *i
            thetadeg = math.degrees(theta)
            #print('Theta ={}, r ={}, thetadeg = {}'.format(theta,r,thetadeg))
            if angle_max<thetadeg or thetadeg<angle_min : #viewing angle
                if r>r_min and r<r_max :   #ranges
                    x= r * math.cos(theta)      #complicated math operation
                    y= r * math.sin(theta)      
                    self.sum_x = self.sum_x +x  
                    self.sum_y = self.sum_y +y
                    tour +=1
        if tour == 0 :  # For debug when tour = 0 
            msg_twist.linear.x = 0.0
            msg_twist.angular.z = 0.0
        else :  #Loop primarily used

            bari_x = self.sum_x / tour #determine the barycenter
            bari_y = self.sum_y /tour
            print('bari_x ={}, bari_y ={}'.format(bari_x, bari_y))
            print('time ={}'.format(msg.scan_time))

            self.x_centre = (r_max + r_min)/2   #determine the center of perception area

            delta_x = bari_x -self.x_centre     # To control the distance in x
            delta_y = bari_y                    # To control the direction

            self.lin_vel = delta_x * factor_lin     #To make the accelerations fluently and smoother
            self.rot_vel = delta_y * factor_rot     #To avoid robot oscillations

            msg_twist.linear.x = self.lin_vel       #To set the parameters to send
            msg_twist.angular.z = self.rot_vel 

            #self.get_logger().info ("Fin de boucle de calcul")          #Step debug 3
            print('delta_x ={}, delta_y ={}'.format(delta_x,delta_y))    #Be sure of values 
        print('lin_vel ={}, rot_vel ={}'.format(self.lin_vel, self.rot_vel))    

      

        self.publisher.publish(msg_twist) #Send datas to robot
        



       #Try something
        #twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        #pub.publish(twist)





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
