import mujoco
import mujoco.viewer
import numpy as np

# Load MuJoCo model and create corresponding data
model = mujoco.MjModel.from_xml_path('scene.xml')
data = mujoco.MjData(model)

# Launch viewer in passive mode without UI panels
with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
    # Run simulation while viewer is active
    while viewer.is_running():
        mujoco.mj_step(model, data)  # Step the simulation
        viewer.sync()                # Update the viewer