import mujoco
import mujoco.viewer
import random

def main():

    # MuJoCo initialization
    model = mujoco.MjModel.from_xml_path('scene.xml')
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()