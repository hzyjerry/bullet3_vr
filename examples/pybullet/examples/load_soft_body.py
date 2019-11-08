import pybullet as p
from time import sleep

physicsClient = p.connect(p.SHARED_MEMORY)

p.setGravity(0, 0, 3)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf", [1, 1, 0])
bunnyId = p.loadSoftBody("hospitalgown_adaptivereduce.obj", basePosition=[0,0,1])
#meshData = p.getMeshData(bunnyId)
#print("meshData=",meshData)
p.loadURDF("cube_small.urdf", [0, 0, 1])
useRealTimeSimulation = 1

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

# print(p.getDynamicsInfo(planeId, -1))
# print(p.getDynamicsInfo(bunnyId, 0))
# print(p.getDynamicsInfo(boxId, -1))

while p.isConnected():
  p.setGravity(0, 0, 0)
  if not useRealTimeSimulation:
  	p.stepSimulation()
    # sleep(0.01)  # Time in seconds.
    #p.getCameraImage(320,200,renderer=p.ER_BULLET_HARDWARE_OPENGL )
  # else:
  #   p.stepSimulation()
