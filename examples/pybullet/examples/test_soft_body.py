import pybullet as p
import numpy as np
import time

physicsClient = p.connect(p.GUI)

p.setGravity(0, 0, 0)

# p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf", [0,0,0])

# block_collision = p.createCollisionShape(p.GEOM_CYLINDER, halfExtents=[size, size, size])
# block_visual = p.createVisualShape(p.GEOM_CYLINDER, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
# block = p.createMultiBody(baseMass=5, baseCollisionShapeIndex=block_collision, baseVisualShapeIndex=block_visual, basePosition=[0, 0, 3], useMaximalCoordinates=1)

# cloth = p.loadCloth("fullgown_midpoint2_reduced.obj", basePosition=[1.0, 0, 0.0], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), scale=5, mass=0.23, collisionMargin=0.06, rgbaColor=np.array([139./256., 195./256., 74./256., 0.6]), rgbaLineColor=np.array([197./256., 225./256., 165./256., 1]), useBendingSprings=1, springElasticStiffness=0.1, useSelfCollision=1)

cloth = p.loadSoftBody("sphere8.obj")


x, y, z, cx, cy, cz, fx, fy, fz = p.getSoftBodyData(cloth)
mesh_points = np.concatenate([np.expand_dims(x, axis=-1), np.expand_dims(y, axis=-1), np.expand_dims(z, axis=-1)], axis=-1)
print(mesh_points.shape)
# x = [1061, 1615, 1508, 1084, 1707, 1511, 1378, 2150, 1005, 1846, 2676, 1616]
# min_x = min(mesh_points[200][0], mesh_points[1450][0])
# min_y = min(mesh_points[200][0], mesh_points[1450][0])
# max_x = max(mesh_points[200][0], mesh_points[1450][0])
# max_y = max(mesh_points[200][0], mesh_points[1450][0])
print(mesh_points[400])
print(mesh_points[723])
print(mesh_points[496])
print(mesh_points[434])


for i in range(mesh_points.shape[0]):
	if (mesh_points[i][2]<0.92 and mesh_points[i][2]>0.7):
		p.addUserDebugText(str(i), mesh_points[i], [1,0,0], 1)



# cloth = p.loadCloth("hospitalgown_reduced_1104verts_sorted.obj", basePosition=[0,0,1], baseOrientation=p.getQuaternionFromEuler([np.pi/2.0, 0, -np.pi/2.0]), scale=1, mass=0.23, collisionMargin=0.06, rgbaColor=np.array([139./256., 195./256., 74./256., 0.6]))

# cloth_attachment1 = p.createMultiBody(baseMass=0.0, basePosition=[0.3, 0, 1], useMaximalCoordinates=1)
# cloth_attachment2 = p.createMultiBody(baseMass=0.0, basePosition=[0.3, 0.3, 1], useMaximalCoordinates=1)
# cloth_attachment3 = p.createMultiBody(baseMass=0.0, basePosition=[0, 0, 1], useMaximalCoordinates=1)
# cloth_attachment4 = p.createMultiBody(baseMass=0.0, basePosition=[0, 0.3, 1], useMaximalCoordinates=1)
time.sleep(1000)

# p.createSoftBodyAnchor(cloth, nodeIndices=[104, 105], cloth_attachment1)
# p.createSoftBodyAnchor(cloth, 105, cloth_attachment2)
# p.createSoftBodyAnchor(cloth, 930, cloth_attachment3)
# p.createSoftBodyAnchor(cloth, 961, cloth_attachment4)

# self.cloth = p.loadCloth(os.path.join(self.world_creation.directory, 'clothing', 'hospitalgown_reduced_1104verts_sorted.obj'), scale=1.4, mass=0.23, position=np.array([0.07, -0.47, 0.82]) + self.cloth_offset/1.4, orientation=p.getQuaternionFromEuler([np.pi/2.0, 0, -np.pi/2.0], physicsClientId=self.id), bodyAnchorId=self.cloth_attachment, anchors=[104, 105, 930, 961, 532, 725, 931, 106], collisionMargin=0.04, rgbaColor=np.array([139./256., 195./256., 74./256., 0.6]), rgbaLineColor=np.array([197./256., 225./256., 165./256., 1]), physicsClientId=self.id)
# p.clothParams(self.cloth, kLST=0.01, kAST=1.0, kVST=1.0, kDP=0.001, kDG=10, kDF=0.25, kCHR=1.0, kKHR=1.0, kAHR=1.0, piterations=5, physicsClientId=self.id)

# time.sleep(10)
# boxId = p.loadURDF("cube_small.urdf", [.1,.1,2],useMaximalCoordinates = True)

# "softBodyBodyUniqueId", "nodeIndex",
# 							 "bodyUniqueId", "linkIndex", "bodyFramePosition",
# 							 "physicsClientId",
# p.createSoftBodyAnchor(clothId ,3,boxId,-1, [0.5,-0.5,0])
# p.createSoftBodyAnchor(clothId ,2,boxId,-1, [-0.5,-0.5,0])

# cloth2 = p.loadSoftBody("hospitalgown_adaptivereduce.obj", basePosition=[0,0,0], scale=1, mass=0, collisionMargin=0.05)
# "fileName", "basePosition", "baseOrientation", "scale", "mass", "collisionMargin", "physicsClientId", "useMassSpring", "useBendingSprings", "useNeoHookean", "springElasticStiffness", "springDampingStiffness", "NeoHookeanMu", "NeoHookeanLambda", "NeoHookeanDamping", "frictionCoeff", "useFaceContact", "useSelfCollision", NULL};

while p.isConnected():
    p.setGravity(0, 0, -10)
    p.stepSimulation()