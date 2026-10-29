import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float32
rclpy.init()
n=Node('probe')
d={'b':[], 's':[], 'n':[]}
n.create_subscription(Float32,'/bot2/power/battery_v',lambda m:d['b'].append((time.time(),m.data)),10)
n.create_subscription(Float32,'/bot2/power/solar_v',lambda m:d['s'].append((time.time(),m.data)),10)
n.create_subscription(Float32,'/bot2/solar/servo_now',lambda m:d['n'].append((time.time(),m.data)),10)
t=time.time()
while time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.1)
for k,v in d.items(): print(k,len(v),v[-3:])
