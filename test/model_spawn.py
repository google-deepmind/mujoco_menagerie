import mujoco
from robot_descriptions.loaders.mujoco import load_robot_description
import mujoco.viewer

# 로봇 모델 로드
model = load_robot_description("panda_mj_description")
data = mujoco.MjData(model)

# Viewer 실행
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
