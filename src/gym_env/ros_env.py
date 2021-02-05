import gym
import rospy
import numpy as np

from collections import OrderedDict
from gym_env.msg import SensorMessage, ActuatorMessage
from gym_env.srv import RegisterSpaces, GetReward, StepEnv, ResetEnv, CloseEnv
from gym import spaces

class ROSEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, sensors, actuators, reward, environment):
        super(ROSEnv, self).__init__()

        #self.sensors = sensors
        #self.actutors = actuators
        #self.reward = reward
        #self.environment = environment
        
        # Init Sensors
        rospy.loginfo("Initializing Sensors")

        obs_dict = OrderedDict()
        self.observations = OrderedDict()
        for sensor in sensors:
            register_sensor = rospy.ServiceProxy(sensor, RegisterSpaces)
            low, high, topic = register_sensor()

            obs_dict[sensor] = spaces.Box(list(low), list(high))

            rospy.Subscriber(topic, SensorMessage, self._sensor_callback)
            self.observations[topic] = None
        
        if len(obs_dict) == 1:
            self.observation_space = obs_dict.values()[0]
        else:
            self.observation_space = spaces.Dict(spaces=obs_dict)

        rospy.loginfo("Observations space: {0}", self.observation_space)
        
        # Init Actuators
        rospy.loginfo("Initializing Actuators")

        act_dict = OrderedDict()
        self.actuators = OrderedDict()
        for actuator in actuators:
            register_actuator = rospy.ServiceProxy(actuator, RegisterSpaces)
            low, high, topic = register_actuator()

            act_dict[actuator] = spaces.Box(list(low), list(high))

            self.actuators[topic] = rospy.Publisher(topic, ActuatorMessage)
        
        if len(act_dict) == 1:
            self.action_space = act_dict.values()[0]
        else:
            self.action_space = spaces.Dict(spaces=act_dict)

        rospy.loginfo("Action space: {0}", self.action_space)

        self.get_reward = rospy.ServiceProxy(reward, GetReward)
        self.step_env = rospy.ServiceProxy(environment, StepEnv)
        self.reset_env = rospy.ServiceProxy(environment, ResetEnv)
        self.close_env = rospy.ServiceProxy(environment, CloseEnv)

    def step(self, action):

        for (topic, act) in action:
            self.actuators[topic].publish(ActuatorMessage(act))

        # How do reward, done and obs relate to each other? What is the correct order here?
        done = self.step_env()
        reward = self.get_reward()

        observation = self._get_observations()

        return observation, reward, done, {}
    
    def reset(self):

        self.reset_env()
        observation = self._get_observations()
        
        return observation
    
    def render(self, mode='human'):
        pass
    
    def close(self):
        self.close_env()
        rospy.signal_shutdown("Shutdown requested by Gym")

    def _sensor_callback(self, data):
        self.observations[data._connection_header['topic']] = data.data
    
    def _get_observations(self):
        obs_list = []
        for obs in self.observations.values():
            obs_list += obs
        return np.array(obs_list)
