###simulationprereactivemotor.py
import pybullet as p
from multiprocessing import Pool

#for the makemoiuntains
import random 
import math
import time

import pybullet_data
import numpy as np

import os

import uuid

import tempfile

import datetime

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.sim_id = sim_id

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.physicsClientId)
        p.setAdditionalSearchPath('shapes/', physicsClientId=self.physicsClientId) # Path for gaussian_pyramid.urdf

    #moved the functions to generate the environment from cw-envt.py into here
    def make_mountain(self,num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5,pid = 0):
        def gaussian(x, y, sigma=arena_size/4):
            """Return the height of the mountain at position (x, y) using a Gaussian function."""
            return mountain_height * math.exp(-((x**2 + y**2) / (2 * sigma**2)))

        for _ in range(num_rocks):
            x = random.uniform(-1 * arena_size/2, arena_size/2)
            y = random.uniform(-1 * arena_size/2, arena_size/2)
            z = gaussian(x, y)  # Height determined by the Gaussian function

            # Adjust the size of the rocks based on height. Higher rocks (closer to the peak) will be smaller.
            size_factor = 1 - (z / mountain_height)
            size = random.uniform(0.1, max_size) * size_factor

            orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)], physicsClientId=pid)
            rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size], physicsClientId=pid)
            rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1], physicsClientId=pid)
            rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation, physicsClientId=pid)

    def make_rocks(self,num_rocks=100, max_size=0.25, arena_size=10,pid = 0):
        for _ in range(num_rocks):
            x = random.uniform(-1 * arena_size/2, arena_size/2)
            y = random.uniform(-1 * arena_size/2, arena_size/2)
            z = 0.5  # Adjust based on your needs
            size = random.uniform(0.1,max_size)
            orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)], physicsClientId=pid)
            rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size], physicsClientId=pid)
            rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1], physicsClientId=pid)
            rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation, physicsClientId=pid)

    def make_arena(self,arena_size=10, wall_height=1,pid = 0):
        wall_thickness = 0.5
        floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], physicsClientId=pid)
        floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1], physicsClientId=pid)
        floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness], physicsClientId=pid)

        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], physicsClientId=pid)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)  # Gray walls

        # Create four walls
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2], physicsClientId=pid)

        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], physicsClientId=pid)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=pid)  # Gray walls

        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2], physicsClientId=pid)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2], physicsClientId=pid)

    def run_creature(self, cr, iterations=2400,current_iter = 0):
        print(f"time starting: {datetime.datetime.now()}")

        pid = self.physicsClientId
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)

        p.setGravity(0, 0, -10, physicsClientId=pid)


        arena_size = 23
        Simulation.make_arena(self,arena_size=arena_size,pid = pid)

        mountain_position = (0, 0, -1)
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
        p.setAdditionalSearchPath('shapes/')

        mountain_id = p.loadURDF("gaussian_pyramid_larger_wider.urdf", mountain_position, mountain_orientation, useFixedBase=1,physicsClientId=pid)
        guassian_pyramid_height = 6

        #Touched this
        max_height = guassian_pyramid_height + 3.0 # e.g., 3 meters above the peak is too high
        min_height = -1.5 # e.g., 5 meters below the floor is out of bounds
        
        # This will penalize going outside the square arena or flying too high/falling too low
        x_bound_min = -(arena_size+2/2)
        x_bound_max = (arena_size+2/2)
        y_bound_min = -(arena_size+2/2)
        y_bound_max = (arena_size+2/2)

        #old method
        # xml_file = 'temp' + str(self.sim_id) + '.urdf'
        # xml_str = cr.to_xml()
        # with open(xml_file, 'w') as f:
        #     f.write(xml_str)

        #shared temporary file
        #each call first writes the current creature's URDF to 'temp0.urdf', then immediately tries to load it.
        #there might be a very slight delay before the file system completely flushes and closes the file, 
        # making it fully ready for PyBullet to read reliably.
        # if p.loadURDF tries to read the file while it's still being written or is in an unstable state, it might fail or load a corrupted URDF, leading to an invalid cid. 
        # the getBasePositionAndOrientation then fails because cid isn't a valid object ID.

        #to minimised invalid cid
        unique_filename = f'temp_{self.sim_id}_{current_iter}_{uuid.uuid4()}_.urdf' 
        xml_str = cr.to_xml()
        with open(unique_filename, 'w') as f:
            f.write(xml_str)
        
        cid = p.loadURDF(unique_filename, physicsClientId=pid)

        #clean up the temporary file immediately after loading
        os.remove(unique_filename)#Touched this

        #Touched this
        num_joints = p.getNumJoints(cid, physicsClientId=pid)
        # Iterate through all unique pairs of links (including the base link with other links)
        for i in range(-1, num_joints): # -1 represents the base link
            for j in range(i + 1, num_joints):
                if i == j: # Skip if it's the same link
                    continue
                p.setCollisionFilterPair(cid, cid, i, j, 0, physicsClientId=pid) # '0' means disable collision

        #ensure creature's internal position metrics are reset for THIS run#Touched this
        cr.start_position = None
        cr.last_position = None
        cr.closest_position = None 
        #ensures a clean slate before every simulation run

        p.resetBasePositionAndOrientation(cid, [7, 7, 7], [0, 0, 0, 1], physicsClientId=pid)

        #initialise creature's position tracking#Touched this
        try:
            init_pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
        except p.error as e:
            #this is to catch the invalid cid so it dont suudenly stop the codes
            print(f"ERROR: getBasePositionAndOrientation failed for CID {cid} (Iteration {current_iter}): {e}")
            print(f"This indicates the creature was loaded but is unstable or PyBullet's state is corrupted.")
            # Assign a very poor fitness for this creature as it cannot be evaluated
            cr.start_position = [0,0,-1000] 
            cr.last_position = [0,0,-1000]
            cr.closest_position = [0,0,-1000]
            return # Exit run_creature early

        #since each craeture is spawned in the air, this checks if the creature have landed on the arena yet#Touched this
        touched_ground = False
        exit_iter = 0
        while touched_ground == False:
            p.stepSimulation(physicsClientId=pid)

            contacts = p.getContactPoints(bodyA=cid, physicsClientId=pid)
            for contact in contacts:
                if contact[2] == 0:  # bodyB == 0 → contact with world (usually plane or static environment)
                    touched_ground = True
                    start_pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                    cr.update_position(start_pos)
                    cr.closest_position = start_pos
                    #print(f"Creature ID {cid} (Sim {self.sim_id}) currently at: X={start_pos[0]:.2f}, Y={start_pos[1]:.2f}, Z={start_pos[2]:.2f}")
                    break

            exit_iter +=1
            if exit_iter >= 3 * 240 and exit_iter <= ((3 * 240)+3):
                #if the creature fails to stabilise, try one more time 
                p.resetBasePositionAndOrientation(cid, [7, 7, 7], [0, 0, 0, 1], physicsClientId=pid)
            if exit_iter >= 6 * 240:
                #just remove the creature if they still fails
                print(f"Creature {cid} failed to settle after multiple attempts.")
                cr.closest_position = [0,0, -999]
                break

        if touched_ground == True:
            #give few seciong to stabilise itself before starting the training timer
            for _ in range(240):
                p.stepSimulation(physicsClientId=pid)

            for step in range(iterations):
                p.stepSimulation(physicsClientId=pid)

                if (step % 24 == 0) and touched_ground == True:
                    self.update_motors(cid=cid, cr=cr)

                pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
                cr.update_position(pos)

                #boundary cehck#Touched this
                if (pos[0] < x_bound_min or pos[0] > x_bound_max or
                    pos[1] < y_bound_min or pos[1] > y_bound_max or
                    pos[2] > max_height or pos[2] < min_height):
                    
                    print(f"Creature {cid} (Sim {self.sim_id}) went out of bounds at step {step}. Assigning poor fitness.")
                    cr.closest_position = [0, 0, -999]
                    return # End this creature's simulation early

                if np.linalg.norm(np.asarray([0,0,guassian_pyramid_height]) - np.asarray(pos)) < np.linalg.norm(np.asarray([0,0,guassian_pyramid_height]) - np.asarray(cr.closest_position)):
                    cr.closest_position = pos

                if pos[2] > cr.highest_z:
                    cr.highest_z = pos[2]
                
                cr.final_horizontal_pos = pos

            if cr.closest_position is None: # This could happen if it never settled or always stayed out of bounds
                cr.closest_position = [0,0, -999] # Default to worst if somehow not set
        
    
    def update_motors(self, cid, cr):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        for jid in range(p.getNumJoints(cid,
                                        physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            p.setJointMotorControl2(cid, jid, 
                    controlMode=p.VELOCITY_CONTROL, 
                    targetVelocity=m.get_output(), 
                    force = 10, #default 5
                    physicsClientId=self.physicsClientId)
        

    # You can add this to the Simulation class:
    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, 2400) 


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
