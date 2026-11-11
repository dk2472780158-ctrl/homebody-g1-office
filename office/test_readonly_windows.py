import ast,io,json,time,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

def load_functions(path,names,namespace):
    tree=ast.parse(Path(path).read_text(encoding='utf-8'))
    chosen=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=chosen,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

class ReadonlyWindowChecks(unittest.TestCase):
    def test_slow_clouds_exclude_old_frames(self):
        f=load_functions('sensors/live_ros_observer.py',{'bounded_recent'},{'np':np})['bounded_recent']
        times=[(i*.5,i*.5) for i in range(9)]
        points=[np.array([[i,0,0]]) for i in range(9)]
        recent,n,stamp,received=f(points,times)
        self.assertEqual(n,5);self.assertEqual(stamp,2.);self.assertEqual(received,2.)
        self.assertEqual(recent[:,0].tolist(),[4,5,6,7,8])
    def test_receive_gap_cannot_keep_old_source_frame(self):
        f=load_functions('sensors/live_ros_observer.py',{'bounded_recent'},{'np':np})['bounded_recent']
        recent,n,stamp,received=f([np.ones((1,3)),np.zeros((1,3))],[(1.,0.),(1.1,3.)])
        self.assertEqual(n,1);self.assertEqual(received,3.)
    def test_float32_odom_can_be_logged(self):
        out=io.StringIO()
        ns=load_functions('g1_motion_trace_readonly.py',{'write','odom'},
            {'json':json,'time':time,'out':out,'active_until':time.monotonic()+1})
        ns['odom'](SimpleNamespace(position=np.array([1,2,3],np.float32),velocity=np.zeros(3,np.float32),yaw_speed=np.float32(0)))
        self.assertEqual(json.loads(out.getvalue())['position'],[1.,2.,3.])

if __name__=='__main__':unittest.main()
