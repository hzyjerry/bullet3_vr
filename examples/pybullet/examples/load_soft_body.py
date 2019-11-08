import pybullet as p
from time import sleep

physicsClient = p.connect(p.SHARED_MEMORY)

p.setGravity(0, 0, 3)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf", [1, 1, 0])
cloth_attachment = p.createMultiBody(baseMass=0.01, basePosition=[-1, 0, 2], useMaximalCoordinates=1)
bunnyId = p.loadStableCloth("hospitalgown_adaptivereduce.obj", basePosition=[0,0,1], baseOrientation=[0,0,0,1], scale=1, mass=1, collisionMargin=0.01, bodyAnchorId=cloth_attachment, anchors=[99, 100, 101], physicsClientId=physicsClient)
# bunnyId = p.loadSoftBody("bunny.obj")
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
