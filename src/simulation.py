import time
import mujoco
import mujoco.viewer
import math

class Simulation:

    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

    def run(self):
        with mujoco.viewer.launch_passive(
            self.model,
            self.data
        ) as viewer:

            while viewer.is_running():
                start = time.time()
                self._apply_control()
                mujoco.mj_step(self.model, self.data)
                self._read_sensors()
                viewer.sync()

                elapsed = time.time() - start
                remaining = self.model.opt.timestep - elapsed

                if remaining > 0:
                    time.sleep(remaining)


    def _apply_control(self):
        dpos = 2.0
        hover = 2.4525
        p = 5.0
        d = 2.0
        pos = self.data.qpos[2]
        vel = self.data.qvel[2] 

        p_error = dpos - pos
        d_error = 0.0 - vel

        power_correction = p * p_error + d * d_error

        # if pos < 1.0:
        #     if vel > 1.5:
        #         power = hover
        #     else:
        #         power = 3.0  
        # else:
        #     if vel > 0.1:  
        #         power = 1.5
        #     elif vel < -0.1:
        #         power = 3.0
        #     else:
        #         power = hover  

        for i in range(4):
            self.data.ctrl[i] = hover + power_correction


    def _read_sensors(self):
        pos = self.data.qpos[2]
        vel = self.data.qvel[2]
        print(f"Czas: {self.data.time:.2f}s | pos: {pos:.2f} | vel: {vel:.2f}")