#!/usr/bin/env python3
"""Two isolated ROS masters; explicit observation uplink and clock downlink."""
import argparse
import concurrent.futures
import json
from pathlib import Path
import subprocess
import tempfile
import uuid

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--binary', required=True, type=Path)
parser.add_argument('--image', required=True)
parser.add_argument('--result', required=True, type=Path)
args = parser.parse_args()
network = 'xgc-observation-' + uuid.uuid4().hex[:8]
root = Path(tempfile.mkdtemp(prefix=network + '-'))
root.chmod(0o777)
names = []

def call(argv):
    return subprocess.check_output(argv, text=True).strip()

probe = r'''
import json,sys,time,xmlrpc.client
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
role=sys.argv[1];rospy.init_node('observation_probe')
received={};messages={}
pose=PoseStamped();pose.header.stamp=rospy.Time(42,123);pose.header.frame_id='map';pose.pose.position.x=1.25;pose.pose.orientation.w=1
measurement=PoseStamped();measurement.header.stamp=pose.header.stamp;measurement.header.frame_id='world';measurement.pose.position.x=2.5;measurement.pose.orientation.w=1
state=State();state.armed=True;state.mode='OFFBOARD'
extended=ExtendedState();extended.landed_state=2
path=Path();path.header=pose.header;path.poses=[pose]
clock=Clock();clock.clock=rospy.Time(42,123)
rows=[('/uav1/mavros/local_position/pose',PoseStamped,pose),('/uav1/pose',PoseStamped,measurement),('/uav1/mavros/state',State,state),('/uav1/mavros/extended_state',ExtendedState,extended),('/uav1/alg/predicted_trajectory',Path,path)]
def accept(topic,msg):received[topic]=received.get(topic,0)+1;messages[topic]=msg
imports=rows if role=='station' else [('/test/clock',Clock,clock)]
exports=rows if role=='robot' else [('/test/clock',Clock,clock)]
subs=[rospy.Subscriber(t,k,lambda msg,t=t:accept(t,msg),queue_size=1) for t,k,v in imports]
pubs=[(rospy.Publisher(t,k,queue_size=1),v) for t,k,v in exports]
private=rospy.Publisher('/robot_private_pose',PoseStamped,queue_size=1) if role=='robot' else None
for _ in range(60):
 for pub,msg in pubs:pub.publish(msg)
 if private:private.publish(pose)
 time.sleep(.1)
for t,k,v in imports:assert received.get(t,0)>0,(role,t,received)
if role=='station':
 assert messages[rows[0][0]].header.stamp==pose.header.stamp
 assert messages[rows[0][0]].pose.position.x==1.25
 assert messages[rows[1][0]].pose.position.x==2.5
 assert messages[rows[2][0]].armed
 assert messages[rows[3][0]].landed_state==2
 assert messages[rows[4][0]].poses[0].header.stamp==pose.header.stamp
 publishers=xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getSystemState('/probe')[2][0]
 assert '/robot_private_pose' not in [r[0] for r in publishers]
else:assert messages['/test/clock'].clock==clock.clock
print(json.dumps({'role':role,'received':received}))
'''
(root/'probe.py').write_text(probe)
start = r'''
set -e
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROS_HOME=/tmp/ros ROS_LOG_DIR=/tmp/log
roscore >/evidence/master-$1.log 2>&1 & master=$!
trap 'kill "$master" "${bridge:-}" 2>/dev/null || true' EXIT
python3 - <<'READY'
import time,xmlrpc.client
until=time.monotonic()+10
while True:
 try:
  assert xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getPid('/probe')[0]==1
  break
 except Exception:
  if time.monotonic()>until:raise
  time.sleep(.05)
READY
rosparam load /evidence/$1.json /observation_bridge
/bridge __name:=observation_bridge >/evidence/bridge-$1.log 2>&1 & bridge=$!
python3 /evidence/probe.py "$1"
'''
try:
    call(['docker','network','create','--internal','--label','xgc2.test=observation',network])
    addresses = {}
    for role in ['station','robot']:
        name=network+'-'+role;names.append(name)
        call(['docker','run','-d','--name',name,'--network',network,'--user','1000:1000','--entrypoint','sleep','--mount',f'type=bind,src={root},dst=/evidence','--mount',f'type=bind,src={args.binary.resolve()},dst=/bridge,readonly',args.image,'infinity'])
        addresses[role]=json.loads(call(['docker','inspect',name]))[0]['NetworkSettings']['Networks'][network]['IPAddress']
    observations=[('mavros/local_position/pose','geometry_msgs/PoseStamped'),('pose','geometry_msgs/PoseStamped'),('mavros/state','mavros_msgs/State'),('mavros/extended_state','mavros_msgs/ExtendedState'),('alg/predicted_trajectory','nav_msgs/Path')]
    for role in ['station','robot']:
        observation=[dict(topic_name='/uav1/'+topic,msg_type=kind,max_freq=10,srcIP='robot',srcPort=31000+i) for i,(topic,kind) in enumerate(observations)]
        clock=[dict(topic_name='/test/clock',msg_type='rosgraph_msgs/Clock',max_freq=10,srcIP='station',srcPort=31010)]
        config={'IP':addresses,'send_topics':observation if role=='robot' else clock,'recv_topics':clock if role=='robot' else observation}
        (root/(role+'.json')).write_text(json.dumps(config))
    def execute(role):
        result=subprocess.run(['docker','exec',network+'-'+role,'bash','-c',start,'fixture',role],text=True,capture_output=True,timeout=25)
        (root/(role+'.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(role+': '+result.stdout+result.stderr+' '+(root/('bridge-'+role+'.log')).read_text())
        return json.loads(result.stdout.splitlines()[-1])
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(execute,['station','robot']))
    result={'scope':'explicit ROS observation bridge between separate container masters; no algorithm or flight claim','results':results,'logs':str(root)}
    args.result.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
finally:
    for name in names:subprocess.run(['docker','rm','-f',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    subprocess.run(['docker','network','rm',network],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
