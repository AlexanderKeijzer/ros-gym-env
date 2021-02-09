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
            rospy.wait_for_service(sensor, 10.0)
            resp = register_sensor()

            obs_dict[sensor] = spaces.Box(np.array(resp.low), np.array(resp.high))

            rospy.Subscriber(resp.topic, SensorMessage, self._sensor_callback)
            self.observations[resp.topic] = np.zeros(len(resp.low))
        
        
        if len(obs_dict) == 1:
            self.observation_space = list(obs_dict.values())[0]
        else:
            self.observation_space = spaces.Dict(obs_dict)
        
        #self.observation_space = spaces.Dict(obs_dict)

        rospy.logwarn("Observations space: %s", str(self.observation_space))
        
        # Init Actuators
        rospy.loginfo("Initializing Actuators")

        act_dict = OrderedDict()
        self.actuators = OrderedDict()
        for actuator in actuators:
            register_actuator = rospy.ServiceProxy(actuator, RegisterSpaces)
            rospy.wait_for_service(actuator, 10.0)
            resp = register_actuator()

            act_dict[actuator] = spaces.Box(np.array(resp.low), np.array(resp.high))

            self.actuators[resp.topic] = rospy.Publisher(resp.topic, ActuatorMessage, queue_size=1)
        
        
        if len(act_dict) == 1:
            self.action_space = list(act_dict.values())[0]
        else:
            self.action_space = spaces.Dict(act_dict)
        
        #self.action_space = spaces.Dict(act_dict)

        rospy.logwarn("Action space: %s", str(self.action_space))

        self.get_reward = rospy.ServiceProxy(reward, GetReward)
        self.step_env = rospy.ServiceProxy(environment + '/step', StepEnv)
        self.reset_env = rospy.ServiceProxy(environment + '/reset', ResetEnv)
        self.close_env = rospy.ServiceProxy(environment + '/close', CloseEnv)

    def step(self, action):

        for act in self.actuators.values():
            if type(self.action_space) == OrderedDict:
                pass
            else:
                act.publish(ActuatorMessage(action))


        #for (topic, act) in action:
        #    self.actuators[topic].publish(ActuatorMessage(act))

        # How do reward, done and obs relate to each other? What is the correct order here?
        done = self.step_env().done
        reward = self.get_reward().reward

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
        self.observations[data._connection_header['topic']] = data.observations
    
    def _get_observations(self):
        return np.array(list(self.observations.values())).flatten()