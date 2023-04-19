import rclpy
from rclpy.node import Node

# from reedsolo import RSCodec, ReedSolomonError

from ackermann_can_interfaces.msg import AckermannDrive
from ackermann_can_interfaces.msg import AckermannDriveStamped

from ackermann_can_interfaces.msg import CAN
from ackermann_can_interfaces.msg import CANStamped
        
class AckermannTranslator(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            CANStamped, 
            '/pub_raw_can', 
            10)
    
        self.i = 0
        self.subscription  # prevent unused variable warnings
        self.publisher
    
    def strbinaryToIntArr(self, inputStr):
        #Splits String of Binary into chunks of 8 and converts to int
        return [int(x,2) for x in [inputStr[i:i+8] for i in range(0, len(inputStr), 8)]]

    def floatToBinaryStr(self, num, exp=4, mant=4,sign=True):
        #Converts float to binary
        returnStr = ""
        if sign:
            #Sign
            if num<0:
                returnStr += "1"
                num = -num
            else:
                returnStr += "0"
        
        #Exponent
        for i in range(exp):
            if num>=2**(exp-i-1):
                returnStr += "1"
                num -= 2**(exp-i-1)
            else:
                returnStr += "0"
        
        #Mantissa
        for i in range(mant):
            if num>=2**(-i-1):
                returnStr += "1"
                num -= 2**(-i-1)
            else:
                returnStr += "0"
        
        return returnStr
    
    def bitstring_to_bytes(self,s):
        return int(s, 2).to_bytes((len(s) + 7) // 8, byteorder='big')
    
    def byteArrtoIntArr(self, byteArr):
        return [int(x) for x in byteArr]

    def addECC(self, data):
        ##### ReedSolo has not been added to RosDep Yet, so this is commented out

        # #Adds ECC to CAN Data
        # byteArr = self.bitstring_to_bytes(self,data)
        # rsc = RSCodec(2)
        # RSEncodedExample = rsc.encode(byteArr)

        # data = self.byteArrtoIntArr(RSEncodedExample)

        return data
        

    def listener_callback(self, msg):
        ### Format:
        #   Unused Header
        #   Steering Angle: Exp=2, Mant=5
        #   Steering Angle Velocity: 0=Max, else Exp=2, Mant=6
        #   Speed: Exp=3, Mant=3
        #   Acceleration: 0=Max, else Exp=4, Mant=4
        #   Jerk: 0=Max, else Exp=4, Mant=5
        header = "0000"         
        steeringAngle = self.floatToBinaryStr(msg.drive.steering_angle,2,5)                                                                                     
        steeringAngleVelocity = "11111111" if msg.drive.steering_angle_velocity == 0 else self.floatToBinaryStr(msg.drive.steering_angle_velocity,2,6,False)    
        speed = self.floatToBinaryStr(msg.drive.speed,7,3,True)                                                                                                 
        acceleration = "11111111" if msg.drive.acceleration == 0 else self.floatToBinaryStr(msg.drive.acceleration,4,4,False)                                   
        jerk = "111111111" if msg.drive.jerk == 0 else self.floatToBinaryStr(msg.drive.jerk,4,5,False)                                                          

        message = header + steeringAngle + steeringAngleVelocity + speed + acceleration + jerk
        
        # message = self.addECC(message)
        message = self.strbinaryToIntArr(message)+ [0] + [0]


        newMsg = CANStamped()
        newMsg.header.stamp = self.get_clock().now().to_msg()
        newMsg.header.frame_id = 'base_link'
        
        newMsg.can.id = 0                      #TODO SET ID
        newMsg.can.data = message

        #Listener
        self.get_logger().info(f'I heard: {msg.header.stamp} {msg.header.frame_id} {msg.drive.steering_angle} {msg.drive.speed} {msg.drive.acceleration} {msg.drive.jerk}')
        self.get_logger().info(f'I Translated: {message}')
    
        #Publisher
        self.publisher.publish(newMsg)
        self.get_logger().info('Publishing: "%s"' % newMsg.can.data)
        self.i += 1
    


def main(args=None):
    rclpy.init(args=args)


    ackermann_translator = AckermannTranslator()
    rclpy.spin(ackermann_translator)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ackermann_translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
