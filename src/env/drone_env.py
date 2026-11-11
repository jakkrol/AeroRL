import mujoco

class DroneEnv:
    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

    def reset(self):
        # mj_resetData automatycznie zeruje czas (data.time), pozycje (qpos) i prędkości (qvel)
        mujoco.mj_resetData(self.model, self.data)
        
        # Przeliczenie fizyki dla punktu startowego (niezbędne przed pierwszym krokiem!)
        mujoco.mj_forward(self.model, self.data)
        
        pos = self.data.qpos[2]
        vel = self.data.qvel[2]
        state = [pos, vel]
        
        return state

    def step(self, action):
        for i in range(4):
            self.data.ctrl[i] = action[i]
            
        mujoco.mj_step(self.model, self.data)
        
        pos = self.data.qpos[2]
        vel = self.data.qvel[2]
        state = [pos, vel]
        
        return state