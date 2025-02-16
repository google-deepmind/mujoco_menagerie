import mujoco
import mujoco.viewer
import random
import time
import numpy as np

def main():

    model = mujoco.MjModel.from_xml_path('scene.xml')
    data = mujoco.MjData(model)
    target_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "target")
    start_time = time.time()
    
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()