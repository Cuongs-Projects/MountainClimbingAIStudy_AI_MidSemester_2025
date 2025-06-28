###cw-envt-copy.py

################################################################################
#There are two models, the "Final Tuned Model" and the "Extension Model".
#
# To run the Final Tuned Model:
# - uses `import creatureprereactivemotor as creature` and 'import genomeprelimbssymmetry as genome'.
# - modify population accordingly
#
# To run the Extension Model (with symmetric limbs and reactive sensors):
# - as per normal eg 'import creature' and 'import genome'.
#
# Blocks of code I wrote or significantly modified are highlighted in yellow.
################################################################################

import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
# import creatureprereactivemotor as creature

# p.connect(p.GUI)

import random
import pybullet as p
import math

import pybullet as p

#for fitness keeping for graphs
import csv #Touched this
import os #Touched this

import population
import genome
# import genomeprelimbssymmetry as genome
import simulation
# import simulationprereactivemotor as simulation


def masterMain():
    #enter the file location to store the data of the elite creature of each generation
    fitness_log_file_path_name = "extension/fitness_log/extension_elites_1000gen.csv"
    log_file_path = os.path.join(fitness_log_file_path_name)

    gene_count = 2 #default 3, final 2 #Touched this
    pop = population.Population(pop_size=100, gene_count=gene_count) #default 10, final 100 #Touched this
    initial_creature_dna = pop.creatures[0].dna

    # save the inital randomised creature
    genome.Genome.to_csv(initial_creature_dna, "extension/initial_bot.csv") #Touched this

    #generate a csv file to contains each generation elite creature's data #Touched this
    with open(log_file_path, 'w', newline='') as csvfile:
        fieldNames = ['Generations','MaxFitness','MinFitness','MeanFitness',
                      'StdFitness','MaxLinks','MinLinks','MeanLinks',
                    'Best_Bot_Fit','Best_Bot_Link','Best_Bot_Distance_to_Peak']
        writer = csv.DictWriter(csvfile, fieldnames=fieldNames)
        writer.writeheader()
        sim = simulation.Simulation()

        for iteration in range(1000): #testing 500, final 1000 #Touched this
            #to indicate training progress
            print(f"testing iteration no: {iteration}") #Touched this

            for cr in pop.creatures:
                sim.run_creature(cr, 40 * 240, current_iter=iteration) #Touched this
            fits = [ cr.fitness_function() for cr in pop.creatures] #Touched this
            links = [len(cr.get_expanded_links()) for cr in pop.creatures] #total number of independantly moving rigid body segments or parts  

            # retrieve important data #Touched this
            max_fit_log = np.max(fits)
            min_fit_log = np.min(fits)
            mean_fit_log = np.mean(fits)
            std_fit_log = np.std(fits)
            max_link_log = np.round(np.max(links))
            min_link_log = np.round(np.min(links))
            mean_link_log = np.round(np.mean(links))
            print(iteration, "fittest:", max_fit_log, "mean:", mean_fit_log, "mean links", mean_link_log, "max links",max_link_log)

            #since the modified fitness function for the extension creature often output negative number,
            #I have to normalised it, so pop.creatures[p1_ind] can work
            # eg [-3,-7,-1] --> [4.000001 , 0.000001 , 6.000001] --> rounded off later in the process = [4 , 0 , 6]
            normalised_fits = [f - min_fit_log + 1e-6 for f in fits] #Touched this
            fit_map = population.Population.get_fitness_map(normalised_fits)

            new_creatures = []
            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                dna = genome.Genome.crossover(p1.dna, p2.dna)


                dna = genome.Genome.point_mutate(dna, rate=0.05, amount=0.25)#Touched this
                dna = genome.Genome.shrink_mutate(dna, rate=0.3) #chance to remove a gene (link).#Touched this
                dna = genome.Genome.grow_mutate(dna, rate=0.15) #chance to add a gene (link).#Touched this
                #PMR - default 0.1 n 0.25, final 0.05 n 0.25
                #SMR - default 0.25, Final 0.3 or 0.25
                #GMR - default 0.1, final 0.15

                cr = creature.Creature(gene_count)
                cr.update_dna(dna)
                new_creatures.append(cr)

            #identify the elite creature
            max_fit = np.max(fits)
            for cr, fit, link in zip(pop.creatures,fits,links):
                if fit == max_fit:  
                    new_cr = creature.Creature(gene_count)
                    new_cr.update_dna(cr.dna)

                    creature_fit_log = fit#Touched this
                    creature_link_log = link
                    creature_distance_to_peak = cr.distance_to_peak()
                    print(f"Best distance from moutain peak: {creature_distance_to_peak}")
                    print(f"best creature data: {cr.highest_z}, {cr.last_position}")

                    new_creatures[0] = new_cr

                    #to save each generation elite creature for qualitative review.
                    filename = "extension/elite_"+str(iteration)+".csv"#Touched this
                    genome.Genome.to_csv(cr.dna, filename)#Touched this
                    break
            pop.creatures = new_creatures
            writer.writerow({#Touched this
                        'Generations': iteration,
                        'MaxFitness': max_fit_log,
                        'MinFitness' : min_fit_log,
                        'MeanFitness': mean_fit_log,
                        'StdFitness': std_fit_log,
                        'MaxLinks': max_link_log,
                        'MinLinks': min_link_log,
                        'MeanLinks': mean_link_log,
                        'Best_Bot_Fit':creature_fit_log,
                        'Best_Bot_Link':creature_link_log,
                        'Best_Bot_Distance_to_Peak':creature_distance_to_peak
            })
    
    csvfile.close()#Touched this
    print(f"\nGA simulation finished. Results saved to {log_file_path}")

if __name__ == "__main__": 
    masterMain()