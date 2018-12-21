# Robomaster-AI-Challenge-2019

## Installation

1. Install OpenAI Gym with `pip install gym` and OpenCV with `pip install opencv-python`.

2. *This will be fixed once we improve the repo structure*
Clone the repo anywhere, then copy the entire `/DJI` folder into `<python-packages-root-directory>/gym/env/`.

3. In `<python-packages-root-directory>/gym/env/__init__.py`, add the following code parallel to similar clauses:

`register(
    id='Robomaster-v0',
    entry_point='gym.envs.DJI:RobomasterEnv',
    max_episode_steps=9999,
    reward_threshold=90.0,
)`

## Running

To run the simulation, execute the python script `.../DJI/robot_play.py`. For now you should see a red and a blue robot. The blue robot shall take a few rounds of patrol before coming to a complete stop.


Updated 12/21/2018
