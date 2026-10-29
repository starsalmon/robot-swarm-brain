import rclpy, time, json, statistics as st
from rclpy.node import Node
from std_msgs.msg import Float32
rclpy.init()
n=Node('aimtest')
S={'b':[], 's':[], 'n':[]}
n.create_subscription(Float32,'/bot2/power/battery_v',lambda m:S['b'].append((time.time(),m.data)),50)
n.create_subscription(Float32,'/bot2/power/solar_v',lambda m:S['s'].append((time.time(),m.data)),50)
n.create_subscription(Float32,'/bot2/solar/servo_now',lambda m:S['n'].append((time.time(),m.data)),50)
pub=n.create_publisher(Float32,'/bot2/solar/servo_deg',10)
def spin(sec):
    t=time.time()
    while time.time()-t<sec: rclpy.spin_once(n,timeout_sec=0.05)
t0=time.time()
while pub.get_subscription_count()<1 and time.time()-t0<10: spin(0.2)
print('subs',pub.get_subscription_count(),flush=True)
blocks=[]
def go(ang,dwell,avg,phase):
    m=Float32(); m.data=float(ang)
    tc=time.time(); nn0=len(S['n'])
    pub.publish(m); spin(1.0); pub.publish(m)
    spin(dwell-1.0)
    te=time.time()
    ts=te-avg
    b=[v for t,v in S['b'] if t>=ts]; s=[v for t,v in S['s'] if t>=ts]
    echo=[v for t,v in S['n'][nn0:]]
    r=dict(phase=phase,angle=ang,t=tc,mean_b=st.mean(b),sd_b=st.pstdev(b),n_b=len(b),mean_s=st.mean(s),sd_s=st.pstdev(s),echo=echo)
    blocks.append(r)
    print(json.dumps(r),flush=True)
    json.dump(dict(blocks=blocks,raw=S),open('/tmp/aim_raw.json','w'))
# park to 90 and settle
go(90,15,6,'pre')
D=14;A=6
fwd=list(range(45,126,10))
for a in fwd: go(a,D,A,'fwd')
for a in reversed(fwd): go(a,D,A,'rev')
mean={}
for a in fwd:
    v=[r['mean_b'] for r in blocks if r['angle']==a and r['phase'] in('fwd','rev')]
    mean[a]=st.mean(v)
best0=max(mean,key=mean.get)
print('coarse mean',mean,'best0',best0,flush=True)
fine=[a for a in range(best0-10,best0+11,5) if 45<=a<=125]
for a in fine: go(a,D,A,'fine')
for a in reversed(fine): go(a,D,A,'fine_rev')
fm={}
for a in fine:
    v=[r['mean_b'] for r in blocks if r['angle']==a and r['phase'].startswith('fine')]
    fm[a]=st.mean(v)
best=max(fm,key=fm.get)
ref=90 if abs(best-90)>=5 else 65
print('fine mean',fm,'best',best,'ref',ref,flush=True)
for i in range(4):
    go(ref,22,10,'AB_ref'); go(best,22,10,'AB_best')
json.dump(dict(blocks=blocks,raw=S,best=best,ref=ref),open('/tmp/aim_raw.json','w'))
go(best,5,3,'park')
print('DONE best',best,flush=True)
