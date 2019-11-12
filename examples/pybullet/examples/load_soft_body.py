import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.setGravity(0, 0, 0)
planeId = p.loadURDF("plane.urdf")
# boxId = p.loadURDF("cube.urdf", [1, 1, 0])
cloth_attachment = p.createMultiBody(baseMass=0, basePosition=[0, 0, 2], useMaximalCoordinates=1)
bunnyId = p.loadStableCloth("hospitalgown_adaptivereduce.obj", basePosition=[0, 0, 2], baseOrientation=[0,0,0,1], scale=1, mass=1, collisionMargin=0.1, bodyAnchorId=cloth_attachment, anchors=[19, 20, 80, 81, 140, 141, 220, 221], physicsClientId=physicsClient)
# bunnyId = p.loadSoftBody("bunny.obj", basePosition=[0,0,1], baseOrientation=[0,0,0,1])
#meshData = p.getMeshData(bunnyId)
#print("meshData=",meshData)
# p.loadURDF("cube_small.urdf", [0, 0, 2])

size = 0.05
block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
block = p.createMultiBody(baseMass=7, baseCollisionShapeIndex=block_collision, baseVisualShapeIndex=block_visual, basePosition=[0, 0, 2.5], useMaximalCoordinates=1)

# p.setRealTimeSimulation(1)
while p.isConnected():
	p.setGravity(0,0,-9.81, block)
	p.stepSimulation()
