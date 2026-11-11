import time
import mujoco.viewer
from src.env.drone_env import DroneEnv

def main():
    env = DroneEnv("assets/scenes/playground.xml") 
    state = env.reset()

    dpos = 2.0
    hover = 2.4525
    p = 5.0
    d = 2.0

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        
        while viewer.is_running():
            start_time = time.time()
            
            pos = state[0]
            vel = state[1]

            p_error = dpos - pos
            d_error = 0.0 - vel
            power_correction = p * p_error + d * d_error

            corrected_power = hover + power_correction
            
            action = [corrected_power, corrected_power, corrected_power, corrected_power]

            state = env.step(action)

            print(f"Czas: {env.data.time:.2f}s | pos: {pos:.2f} | vel: {vel:.2f}")

            viewer.sync()

            elapsed = time.time() - start_time
            remaining = env.model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

if __name__ == "__main__":
    main()