import time
import pybullet as p
import pybullet_data
import pprint
from math import pi


# Настройки среды
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(
    cameraDistance=2,
    cameraYaw=0,
    cameraPitch=-20,
    cameraTargetPosition=[0, 0, 0.5]
)

# Загрузка моделей робота и стола
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf")
robotStartPos = [0, 0, 2]
robotStartOrientation = p.getQuaternionFromEuler([0, pi, 0])
robotPath = "robot_ur10.urdf"
robotId = p.loadURDF(robotPath, robotStartPos, robotStartOrientation)
#objectId = p.loadURDF("random_urdfs/000/000.urdf", 0, 0, 0.6)

#Получение инфо о параметрах робота
joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
joints = []
links = {}

numJoints = p.getNumJoints(robotId)
print('Robot joints: %d'%numJoints)

for joint_id in range(numJoints):
    info = p.getJointInfo(robotId, joint_id)
    data = {
        'jointID': info[0],
        'jointName': info[1].decode('utf-8'),
        'jointType': joint_type[info[2]],
        'jointLowerLimit': info[8],
        'jointUpperLimit': info[9],
        'jointMaxForce': info[10],
        'jointMaxVelocity': info[11]
    }
    print("JOINT INFO:")
    pprint.pprint(data)
    joints.append(data)
    if data['jointType'] != 'FIXED':
        links[data['jointName']] = joint_id


pprint.pprint(links)

# Добавить слайдеры управления
position_control_group = {}
for link_name in links.keys():
    position_control_group[link_name] = p.addUserDebugParameter(link_name, -pi, pi, 0)


print("Start sim")
while True:
    time.sleep(0.01)
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) # ???
    parameter = {}
    for link_name in links.keys():
        parameter[link_name] = p.readUserDebugParameter(position_control_group[link_name])
    num = 0
    for link_name in links.keys():
        joint = joints[links[link_name]]
        p.setJointMotorControl2(robotId, links[link_name], p.POSITION_CONTROL,
                targetPosition=parameter[link_name],
                force=joint['jointMaxForce'],
                maxVelocity=joint['jointMaxVelocity'])
        num += 1
    p.stepSimulation()
print("End sim")
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos, cubeOrn)
p.disconnect()
