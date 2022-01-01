import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
bodyId = p.loadURDF("urdf/Carausius_Morosus.urdf", 0, 0, 2)
# p.loadSDF("world.sdf")

for i in range(5000):    

    i = 0
    time.sleep(1/240)

    
    
    p.stepSimulation()
    i += 1

p.disconnect()