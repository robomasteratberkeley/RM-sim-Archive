import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np

class RM_PlaygroundEnv(gym.Env):
	metadata = {
		'render.modes': ['human', 'rgb_array'],
		'video.frames_per_second': 30
	}
	
	def __init__(self):
		self.min_action = -1.0
		self.max_action = 1.0
		self.min_position = -1.2
		self.max_position = 0.6
		self.max_speed = 0.07
		self.goal_position = 0.45 # was 0.5 in gym, 0.45 in Arnaud de Broissia's version
		self.power = 0.0015

		self.low_state = np.array([self.min_position, -self.max_speed])
		self.high_state = np.array([self.max_position, self.max_speed])

		self.viewer = None

		self.action_space = spaces.Box(low=self.min_action, high=self.max_action, shape=(1,))
		self.observation_space = spaces.Box(low=self.low_state, high=self.high_state)

		self.seed()
		self.reset()

	def seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def step(self, action):

		position = self.state[0]
		velocity = self.state[1]
		force = min(max(action[0], -1.0), 1.0)

		velocity += force*self.power -0.0025 * math.cos(3*position)
		if (velocity > self.max_speed): velocity = self.max_speed
		if (velocity < -self.max_speed): velocity = -self.max_speed
		position += velocity
		if (position > self.max_position): position = self.max_position
		if (position < self.min_position): position = self.min_position
		if (position==self.min_position and velocity<0): velocity = 0

		done = bool(position >= self.goal_position)

		reward = 0
		if done:
			reward = 100.0
		reward-= math.pow(action[0],2)*0.1

		self.state = np.array([position, velocity])
		return self.state, reward, done, {}

	def reset(self):
		self.state = np.array([self.np_random.uniform(low=-0.6, high=-0.4), 0])
		return np.array(self.state)

#	 def get_state(self):
#		 return self.state

	def _height(self, xs):
		return np.sin(3 * xs)*.45+.55

	def render(self, mode='human'):
		screen_width = 800
		screen_height = 500

		world_width = self.max_position - self.min_position
		scale = screen_width/world_width
		carwidth=40
		carheight=20


		if self.viewer is None:
			from gym.envs.classic_control import rendering
			self.viewer = rendering.Viewer(screen_width, screen_height)
			
			l,r = screen_width/2 - 30, screen_width/2 + 30
			t,b = screen_height/2 + 30, screen_height/2 - 30
			bonus_region = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
			self.viewer.add_geom(bonus_region)
			bonus_region.set_color(0.8, 0.8, 0)
			
			team_a_starting_l, team_a_starting_r = 0, 133
			team_a_starting_t, team_a_starting_b = 133, 0
			team_a_starting_region = rendering.FilledPolygon([(team_a_starting_l,team_a_starting_b), 
			(team_a_starting_l,team_a_starting_t), 
			(team_a_starting_r,team_a_starting_t), 
			(team_a_starting_r,team_a_starting_b)])
			self.viewer.add_geom(team_a_starting_region)
			team_a_starting_region.set_color(0, 0, 255)
			
			team_b_starting_l, team_b_starting_r = screen_width - 133, screen_width
			team_b_starting_t, team_b_starting_b = screen_height, screen_height - 133
			team_b_starting_region = rendering.FilledPolygon([(team_b_starting_l,team_b_starting_b), 
			(team_b_starting_l,team_b_starting_t), 
			(team_b_starting_r,team_b_starting_t), 
			(team_b_starting_r,team_b_starting_b)])
			self.viewer.add_geom(team_b_starting_region)
			team_b_starting_region.set_color(255, 0, 0)
			
			obstacle_1_l, obstacle_1_r = 0, 80
			obstacle_1_t, obstacle_1_b = 250, 220
			obstacle_1 = rendering.FilledPolygon([(obstacle_1_l, obstacle_1_b),
			(obstacle_1_l, obstacle_1_t),
			(obstacle_1_r, obstacle_1_t),
			(obstacle_1_r, obstacle_1_b)])
			self.viewer.add_geom(obstacle_1)
			
			obstacle_2_l, obstacle_2_r = 120, 200
			obstacle_2_t, obstacle_2_b = 400, 370
			obstacle_2 = rendering.FilledPolygon([(obstacle_2_l, obstacle_2_b),
			(obstacle_2_l, obstacle_2_t),
			(obstacle_2_r, obstacle_2_t),
			(obstacle_2_r, obstacle_2_b)])
			self.viewer.add_geom(obstacle_2)
			
			obstacle_3_l, obstacle_3_r = 180, 210
			obstacle_3_t, obstacle_3_b = 270, 150
			obstacle_3 = rendering.FilledPolygon([(obstacle_3_l, obstacle_3_b),
			(obstacle_3_l, obstacle_3_t),
			(obstacle_3_r, obstacle_3_t),
			(obstacle_3_r, obstacle_3_b)])
			self.viewer.add_geom(obstacle_3)
			
			obstacle_4_l, obstacle_4_r = 310, 340
			obstacle_4_t, obstacle_4_b = 200, 0
			obstacle_4 = rendering.FilledPolygon([(obstacle_4_l, obstacle_4_b),
			(obstacle_4_l, obstacle_4_t),
			(obstacle_4_r, obstacle_4_t),
			(obstacle_4_r, obstacle_4_b)])
			self.viewer.add_geom(obstacle_4)
			
			obstacle_5_l, obstacle_5_r = screen_width - 80, screen_width
			obstacle_5_t, obstacle_5_b = screen_height - 220, screen_height - 250
			obstacle_5 = rendering.FilledPolygon([(obstacle_5_l, obstacle_5_b),
			(obstacle_5_l, obstacle_5_t),
			(obstacle_5_r, obstacle_5_t),
			(obstacle_5_r, obstacle_5_b)])
			self.viewer.add_geom(obstacle_5)
			
			obstacle_6_l, obstacle_6_r = screen_width - 200, screen_width - 120
			obstacle_6_t, obstacle_6_b = screen_height - 370, screen_height - 400
			obstacle_6 = rendering.FilledPolygon([(obstacle_6_l, obstacle_6_b),
			(obstacle_6_l, obstacle_6_t),
			(obstacle_6_r, obstacle_6_t),
			(obstacle_6_r, obstacle_6_b)])
			self.viewer.add_geom(obstacle_6)
			
			obstacle_7_l, obstacle_7_r = screen_width - 210, screen_width - 180
			obstacle_7_t, obstacle_7_b = screen_height - 150, screen_height - 270
			obstacle_7 = rendering.FilledPolygon([(obstacle_7_l, obstacle_7_b),
			(obstacle_7_l, obstacle_7_t),
			(obstacle_7_r, obstacle_7_t),
			(obstacle_7_r, obstacle_7_b)])
			self.viewer.add_geom(obstacle_7)
			
			obstacle_8_l, obstacle_8_r = screen_width - 340, screen_width - 310
			obstacle_8_t, obstacle_8_b = screen_height, screen_height - 200
			obstacle_8 = rendering.FilledPolygon([(obstacle_8_l, obstacle_8_b),
			(obstacle_8_l, obstacle_8_t),
			(obstacle_8_r, obstacle_8_t),
			(obstacle_8_r, obstacle_8_b)])
			self.viewer.add_geom(obstacle_8)
			
			"""xs = np.linspace(self.min_position, self.max_position, 100)
			ys = self._height(xs)
			xys = list(zip((xs-self.min_position)*scale, ys*scale))

			self.track = rendering.make_polyline(xys)
			self.track.set_linewidth(4)
			self.viewer.add_geom(self.track)

			clearance = 10

			l,r,t,b = -carwidth/2, carwidth/2, carheight, 0
			car = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
			car.add_attr(rendering.Transform(translation=(0, clearance)))
			self.cartrans = rendering.Transform()
			car.add_attr(self.cartrans)
			self.viewer.add_geom(car)
			
			frontwheel = rendering.make_circle(carheight/2.5)
			frontwheel.set_color(.5, .5, .5)
			frontwheel.add_attr(rendering.Transform(translation=(carwidth/4,clearance)))
			frontwheel.add_attr(self.cartrans)
			self.viewer.add_geom(frontwheel)
			
			backwheel = rendering.make_circle(carheight/2.5)
			backwheel.add_attr(rendering.Transform(translation=(-carwidth/4,clearance)))
			backwheel.add_attr(self.cartrans)
			backwheel.set_color(.5, .5, .5)
			self.viewer.add_geom(backwheel)
			
			flagx = (self.goal_position-self.min_position)*scale
			flagy1 = self._height(self.goal_position)*scale
			flagy2 = flagy1 + 50
			flagpole = rendering.Line((flagx, flagy1), (flagx, flagy2))
			self.viewer.add_geom(flagpole)
			
			flag = rendering.FilledPolygon([(flagx, flagy2), (flagx, flagy2-10), (flagx+25, flagy2-5)])
			flag.set_color(.8,.8,0)
			self.viewer.add_geom(flag)

		pos = self.state[0]
		#self.cartrans.set_translation((pos-self.min_position)*scale, self._height(pos)*scale)
		#self.cartrans.set_rotation(math.cos(3 * pos))"""

		return self.viewer.render(return_rgb_array = mode=='rgb_array')

	def close(self):
		if self.viewer:
			self.viewer.close()
			self.viewer = None
		