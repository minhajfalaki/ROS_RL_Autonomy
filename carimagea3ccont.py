import numpy as np
import tensorflow as tf

import time, random, threading
from random import randint
from keras.models import *
from keras.layers import *
from keras import backend as K
import cv2
from game_cont import Game, Reward
import os

#-- constants
resume = False			#for loading weights if the previously trained model has been saved
updatefreq = 50                 #for saving the model
update = 0                      #for saving the model
RUN_TIME = 11000		#time between starting and stopping the thread in seconds
THREADS = 2			#number of environment threads
OPTIMIZERS = 1			#number of optimizer threads
THREAD_DELAY = 0.001		#for yielding between threads

GAMMA = 0.95			#discount factor for future rewards

N_STEP_RETURN = 1		#defines how many steps to look ahead for calculating rewards
GAMMA_N = GAMMA ** N_STEP_RETURN	#calculate future discounted reward factor(gamma_n) for a n step return

EPS_START = 1		#start epsilon value (Exploration Rate)
EPS_STOP  = .001			#end epsilon value
EPS_STEPS = 1000000		#steps to decay epsilon from start value to end value

MIN_BATCH = 64			#min batch required for training
LEARNING_RATE = 1e-2		#learning rate

LOSS_V = .5			# v loss coefficient
LOSS_ENTROPY = .01 		# entropy coefficient

#---------
class Brain:
	train_queue = [ [], [], [], [], [], [], [] ]	
	lock_queue = threading.Lock()		

	def __init__(self):
		self.session = tf.Session()
		K.set_session(self.session)			
		K.manual_variable_initialization(True)		

		self.model = self._build_model()		
		self.graph = self._build_graph(self.model)	

		self.session.run(tf.global_variables_initializer())	
		self.default_graph = tf.get_default_graph()		

		self.default_graph.finalize()	# avoid modifications

	def _build_model(self):			#building the model

		l_input = Input( batch_shape=(None, 150, 200, 1) )
		x = Conv2D(32, (5, 5) , strides=(2, 2), padding='valid', activation='relu', kernel_initializer='glorot_uniform')(l_input)	
		x = MaxPooling2D(pool_size=(2, 2), padding='valid')(x)
		x = Conv2D(32, (3, 3) , strides=(2, 2), padding='valid', activation='relu', kernel_initializer='glorot_uniform')(x)
		x = MaxPooling2D(pool_size=(2, 2), padding='valid')(x)
		out = Flatten()(x)									
		l_dense = Dense(24, activation='relu', kernel_initializer='glorot_uniform')(out)
		l_dense_ = Dense(12, activation='relu', kernel_initializer='glorot_uniform')(l_dense)


		mu = Dense(1,  activation='tanh')(l_dense_)
		sigma = Dense(1,  activation='softplus')(l_dense_)
		#out_actions = Dense(NUM_ACTIONS, activation='softmax')(l_dense_)		#actor output - outputs the actions
		out_value   = Dense(1, activation='linear')(l_dense_)		#critic output - outputs the value

		model = Model(inputs=[l_input], outputs=[mu, sigma, out_value])
		print(model.summary())
		model._make_predict_function()	

		if resume == True:
          	  model.load_weights('carimage-a3cdiscrete.h5')		
		return model

	def _build_graph(self, model):
		s_t = tf.placeholder(tf.float32, shape=(None, 150, 200, 1))		
		a_t = tf.placeholder(tf.float32, shape=(None, 1)) 
		r_t = tf.placeholder(tf.float32, shape=(None, 1))
		p = tf.placeholder(tf.float32, shape=(None, 1))
		e_t = tf.placeholder(tf.float32, shape=(None, 1)) 
		
		mu, sigma, v = model(s_t)	#returns the output for the model

		log_prob = tf.log( tf.reduce_sum(p * a_t, axis=1, keepdims=True) + 1e-10)	
		advantage = r_t - v			

		loss_policy = - log_prob * tf.stop_gradient(advantage)		
		loss_value  = LOSS_V * tf.square(advantage) 	
		entropy = LOSS_ENTROPY * e_t		
		#entropy = LOSS_ENTROPY * tf.reduce_sum(p * tf.log(p + 1e-10), axis=1, keepdims=True)

		loss_total = tf.reduce_mean(loss_policy + loss_value + entropy)	

		optimizer = tf.train.AdamOptimizer(LEARNING_RATE)		
		minimize = optimizer.minimize(loss_total)			

		return s_t, a_t, r_t, p, e_t, minimize

	def optimize(self):
		if len(self.train_queue[0]) < MIN_BATCH:		
			time.sleep(0)	
			return

		with self.lock_queue:
			if len(self.train_queue[0]) < MIN_BATCH:	# more thread could have passed without lock
				return 					# we can't yield inside lock

			s, a, r, s_, s_mask, prob, ent = self.train_queue		
			self.train_queue = [ [], [], [], [], [], [], [] ]
			ts = time.time
			print("time : {}".format(ts))

		s = np.vstack(s)					#process them into solid blocks of numpy arrays
		a = np.vstack(a)
		r = np.vstack(r)
		s_ = np.vstack(s_)
		s_mask = np.vstack(s_mask)
		prob = np.vstack(prob)
		ent = np.vstack(ent)				

		v = self.predict_v(s_)					
		r = r + GAMMA_N * v * s_mask				
		
		s_t, a_t, r_t, p, e_t, minimize = self.graph			#calling the build graph function
		self.session.run(minimize, feed_dict={s_t: s, a_t: a, r_t: r, p: prob, e_t: ent})	

	def train_push(self, s, a, r, s_, p, ent):				
		with self.lock_queue:					#more thread could have passed without lock, Queue consists of 5 arrays
			#if len(self.train_queue[0]) > 1374:
			 # self.train_queue[0].pop(0)
			 # self.train_queue[1].pop(0)
			 # self.train_queue[2].pop(0)
			 # self.train_queue[3].pop(0)
			 # self.train_queue[4].pop(0)
			 # self.train_queue[5].pop(0)
			 # self.train_queue[6].pop(0)
			self.train_queue[0].append(s)			#starting state s
			self.train_queue[1].append(a)			#one-hot encoded taken action a
			self.train_queue[2].append(r)			#discounted n-step return r

			if s_ is None:
				self.train_queue[3].append(NONE_STATE)	
				self.train_queue[4].append(0.)		#a terminal mask with values 1. or 0., indicating whether the s_ is None or not.
			else:	
				self.train_queue[3].append(s_)		
				self.train_queue[4].append(1.)
			self.train_queue[5].append(p)
			self.train_queue[6].append(ent)		

	def predict(self, s):						#predicting policy and value
		with self.default_graph.as_default():			#makes this graph the default graph
			mu, sigma, v = self.model.predict(s.reshape(-1 , 150, 200, 1))
			return mu, sigma, v

	def predict_p(self, s):						#predicting policy
		with self.default_graph.as_default():
			mu, sigma, v = self.model.predict(s.reshape(-1 , 150, 200, 1))		
			return mu, sigma

	def predict_v(self, s):						#predicting value
		with self.default_graph.as_default():
			mu, sigma, v = self.model.predict(s.reshape(-1 , 150, 200, 1))		
			return v

#---------
frames = 0
class Agent:
	def __init__(self, eps_start, eps_end, eps_steps):
		self.eps_start = eps_start				
		self.eps_end   = eps_end				
		self.eps_steps = eps_steps				#self.env = env
		self.memory = []
		self.R = 0.

	def getEpsilon(self):						
		if(frames >= self.eps_steps):
			return self.eps_end				
		else:
			return self.eps_start + frames * (self.eps_end - self.eps_start) / self.eps_steps	# linearly interpolate

	def act(self, s):
		sess = tf.Session()
		s = np.array([s])
		mu, sigma = brain.predict_p(s)	
		normal_dist = tf.contrib.distributions.Normal(mu, sigma)
		ac = normal_dist.sample(1)
		ac = tf.reshape(ac, shape=[1])
		pr = normal_dist.log_prob(ac)
		entr = normal_dist.entropy()
		pr = tf.reshape(pr, shape=[1])
		entr = tf.reshape(entr, shape=[1])
		action, p, ent = sess.run([ac, pr, entr])
		return action, p, ent

	def train(self, s, a, r, s_, p, ent):
		def get_sample(memory, n):				
			s, a, _, _, _, _  = memory[0]
			_, _, _, s_, p, ent = memory[n-1]

			return s, a, self.R, s_, p, ent

		self.memory.append( (s, a, r, s_, p, ent) )		

		self.R = ( self.R + r * GAMMA_N ) / GAMMA		#discounted reward R is computed

		if s_ is None:						#terminal state reached
			while len(self.memory) > 0:			#empty the memory
				n = len(self.memory)
				s, a, r, s_, p, ent = get_sample(self.memory, n)
				brain.train_push(s, a, r, s_, p, ent)

				self.R = ( self.R - self.memory[0][2] ) / GAMMA
				self.memory.pop(0)		

			self.R = 0

		if len(self.memory) >= N_STEP_RETURN:			
			s, a, r, s_, p, ent = get_sample(self.memory, N_STEP_RETURN)
			brain.train_push(s, a, r, s_, p, ent)

			self.R = self.R - self.memory[0][2]
			self.memory.pop(0)	
	
	
		
#---------
class Environment(threading.Thread):
	stop_signal = False

	def __init__(self, car_name, eps_start=EPS_START, eps_end=EPS_STOP, eps_steps=EPS_STEPS):
		threading.Thread.__init__(self)					
		#self.train_data=np.load('norm_dat.npy')
		self.env = Game(car_name,False)	
		self.reward = Reward(car_name,False)
		self.agent = Agent( eps_start, eps_end, eps_steps)	
		self.n = 0						

	def runEpisode(self):
		global update							
		self.env.respawn(True)	
		s = self.reward.returns([0,0])
		R = 0							
		r = 0
		done = False						
		time.sleep(THREAD_DELAY) 				
		while not done:
			print s[1]
			s = np.reshape(s, [-1, 150, 200, 1])	
			a, p, ent = self.agent.act(s)
			retn = self.reward.returns([a,5.56])
			r = retn[2]
			done = self.env.is_episode_finished()
			if done == False:
			  ns = retn[1]
			  ns = np.reshape(ns, [-1, 150, 200, 1])	
			if done == True:
			  ns = None				
	
			self.agent.train(s, a, r, ns, p, ent)
			if done == False:
			  s = ns

			R += r				

			if done or self.stop_signal:
			  update += 1
			  break

		print("END OF EPISODE...Total reward: {}, act {}".format(R, a))
		if update % updatefreq == 0:                      	
			os.remove('carimage-a3c2.h5') if os.path.exists('carimage-a3c2.h5') else None
      			brain.model.save('carimage-a3cdiscrete.h5')

	def run(self):							# runs episodes in loop
		while not self.stop_signal:
			self.runEpisode()

	def stop(self):							#stop signal for stopping from running the environment
		self.stop_signal = True

#---------
class Optimizer(threading.Thread):
	stop_signal = False

	def __init__(self):
		threading.Thread.__init__(self)				

	def run(self):
		while not self.stop_signal:				
			brain.optimize()

	def stop(self):
		self.stop_signal = True					#stop signal

#-- main

with tf.device("/gpu:0"): 
	env_test = Environment("fusion" , eps_start=0., eps_end=0.)		#env. for testing (after training)	

	NUM_ACTIONS = 1
	NONE_STATE = np.zeros((150,200,1))					

	brain = Brain()	# brain is global in A3C				

#envs = [Environment() for i in range(THREADS)]				#creates instances of Environment
with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
	env1 = Environment("fusion")
	env2 = Environment("mkz")
	opts = [Optimizer() for i in range(OPTIMIZERS)]				#creates instances of Optimizer

	for o in opts:								#starts the threads
		o.start()							

	env1.start()
	env2.start()

	time.sleep(RUN_TIME)							#waits for their run time

	env1.stop()
	env2.stop()

	env1.join()
	env2.join()							

	for o in opts:
		o.stop()
	for o in opts:
		o.join()

	print("Training finished")
	env_test.run()		                                                #after training run
