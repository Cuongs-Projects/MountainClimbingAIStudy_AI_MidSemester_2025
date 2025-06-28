# MountainClimbingAIStudy_AI_MidSemester_2025

This repository contains a Python project developed for the Artificial Intelligence Part B (Evolutionary Robotics) module under the University of London BSc Computer Science (Machine Learning and AI) programme. It implements and extends a Genetic Algorithm (GA) framework to evolve virtual creatures capable of climbing a Gaussian mountain within the PyBullet physics simulation environment.

The project explores morphological evolution, articulation control, and bio-inspired extensions to evolve agents exhibiting effective climbing behaviours or morphological exploits.

    Project Tasks & Features:

        - Environment Setup and Simulation:

            * Implements a mountain environment (19x19x7 m) within PyBullet to require genuine climbing behaviour.

            * Includes fitness functions rewarding proximity to the peak, stability, and later penalising undesirable non-climbing strategies.

        - Genetic Algorithm Framework:

            * Population-based evolution with elitism strategy to preserve fittest creatures.

        - Parameter tuning experiments covering:

            * Gene Count (number of initial body links)

            * Point Mutation Rate (PMR)

            * Grow and Shrink Mutation Rates (GMR, SMR)

            * Limb Shape Encoding (cuboid, cylinder, sphere, evolving shapes)

            * Joint Axis Encoding (fixed vs evolving)

            * Commanded Force parameter

        - Final tuned configuration employed a population size of 100, gene_count=2, PMR=0.05, SMR=0.3, GMR=0.15, evolving limb shapes and joint axes, and force=10.

        - Morphological Evolution Experiments:

            * Analysed effects of different limb shapes on performance and exploit emergence.

            * Analysed effects of different limb shapes on performance and exploit emergence.
            Investigated articulation flexibility versus structural compensation (fixed axes leading to “bushy” creatures).

        - Extension Model 
            * Bio-Inspired Enhancements, Bilateral Symmetry:

                - Enforcement to constrain morphological growth towards structured, physically plausible body plans.

            * Reactive touch sensors enabling sensory-motor coupling:

                - Limbs push away when contact is detected, transforming control from blind wiggling to reactive climbing attempts.

            * Disabled sphere limbs to prevent rolling exploits and extended simulation time for evaluation.

        - Simulation and Visualisation:

            * Each creature’s URDF is dynamically saved and loaded with error handling.

            * Simulations demonstrate evolved climbing, sideways exploit strategies, and structural adaptations.

    Technologies Used:

    Python 3

    PyBullet physics simulator

    NumPy

    Standard Python libraries (e.g., os, random, math)

    File Structure:

        creature.py: Core Creature class and fitness calculations, with touch sensor extension.

        genome.py, population.py: Genetic Algorithm implementation, with bilateral symmetry extension.

        prepare_shapes.py: Limb shape preparation.

        simulation.py: Runs the main simulation for final models.

        genomeprelimbssymmetry.py, creatureprereactivemotor.py, simulationprereactivemotor.py: Scripts before implementing the Extension Model.

        requirements.txt: Python dependencies.

        [DONE]AI-MidSem-PartB-Report.pdf: Full report detailing experiments and results.

    Running the Project:

        Install dependencies:

pip install -r requirements.txt

To train the creature with the current files:

python cs-envt-copy.py

To view the selected creature, checkout setup.ipynb
