#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from project_services.srv import OddEvenCheck


class OddEvenCheckServer(Node):
    def __init__(self):
        super().__init__("server_node")
        self.srv = self.create_service(OddEvenCheck, "odd_even_check_service", self.check_odd_even)
    
    def check_odd_even(self, request, response):
        print(f"Request Received to check: {request.number}")

        if request.number%2 == 0:
            response.message = "The number is Even"
        else:
            response.message = "The number is Odd"
      
        return response

def main(args=None):
    rclpy.init()      # Initializes the ROS DDS communication  
    server_node = OddEvenCheckServer()
    print("Odd Even Check Server node is running ...")
    
    try:
        rclpy.spin(server_node) # Keeps the node running until a keyborad key is pressed
    except KeyboardInterrupt:
        print("Terminating the node ...")
        server_node.destroy_node()
        
        
if __name__ == "__main__":
    main()