#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from project_services.srv import OddEvenCheck

class OddEvenCheckClient(Node):
    def __init__(self):
        super().__init__("client_node")
        self.client = self.create_client(OddEvenCheck, "odd_even_check_service")
        self.req = OddEvenCheck.Request() # creates an empty instance of the request part of the service.
    
    def send_odd_even_check_request(self, number):
        self.req.number = int(number)
        self.client.wait_for_service() # the client will wait until the server is available, then it will send the request 
        self.future = self.client.call_async(self.req) # this means that the client sends the request and starts its own execution without waiting for the response.
        rclpy.spin_until_future_complete(self, self.future)
        self.result = self.future.result()
        return self.result

def main(args=None):
    rclpy.init()      # Initializes the ROS DDS communication  
    client_node = OddEvenCheckClient()
    print("Odd Even Check Service Client node is running ...")
    
    try:
        user_input = input("Enter the number which you want verify if it is odd or even: ")
        res = client_node.send_odd_even_check_request(user_input)
        print(f"Server returned:{res}")
    except KeyboardInterrupt:
        print("Terminating the node ...")
        client_node.destroy_node()
        
        
if __name__ == "__main__":
    main()