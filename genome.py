###genome.py
import numpy as np
import copy 
import random

class Genome():
    @staticmethod 
    def get_random_gene(length):
        gene = np.array([np.random.random() for i in range(length)])
        return gene
    
    @staticmethod 
    def get_random_genome(gene_length, gene_count):
        genome = [Genome.get_random_gene(gene_length) for i in range(gene_count)]
        return genome

    @staticmethod
    def get_gene_spec():
        gene_spec =  {"link-shape":{"scale":1}, 
            "link-length": {"scale":2},
            "link-radius": {"scale":1},
            "link-recurrence": {"scale":2}, #was 3, final 2#Touched this

            #determines which central spine link the PAIR attaches to.
            "joint-parent-spine-link": {"scale": 1}, #Touched this
                # scaled_value = gene_value * scale
                # final_index = int(scaled_value * 3)
                #since I only have 3 hard-coded spines at the moment, scale is set to one so that
                #whatever the value genome is used, it will select one of the three spines.

            #controls how far from the center the limb pair is placed (e.g., along the Y-axis)
            "joint-y-offset": {"scale": 1.5}, #Touched this

            #controls the phase offset between the two symmetrical motors. 
            #having the value of 0.5 would mean they move in perfect opposition (like legs).
            #set as 1 for simplicity at the moment
            "control-phase-offset": {"scale": 1.0},#Touched this

            "link-mass": {"scale":1},
            "joint-type": {"scale":1},
            #"joint-parent":{"scale":1}, #no longer relevant
            "joint-axis-xyz": {"scale":1},
            "joint-origin-rpy-1":{"scale":np.pi * 2},
            "joint-origin-rpy-2":{"scale":np.pi * 2},
            "joint-origin-rpy-3":{"scale":np.pi * 2},
            "joint-origin-xyz-1":{"scale":1},
            #"joint-origin-xyz-2":{"scale":1}, #no longer relevant
            "joint-origin-xyz-3":{"scale":1},
            "control-waveform":{"scale":1},
            "control-amp":{"scale":0.25},
            "control-freq":{"scale":1}
            }
        ind = 0
        for key in gene_spec.keys():
            gene_spec[key]["ind"] = ind
            ind = ind + 1
        return gene_spec
    
    @staticmethod
    def get_gene_dict(gene, spec):
        gdict = {}
        for key in spec:
            ind = spec[key]["ind"]
            scale = spec[key]["scale"]
            gdict[key] = gene[ind] * scale
        return gdict

    @staticmethod
    def get_genome_dicts(genome, spec):
        gdicts = []
        for gene in genome:
            gdicts.append(Genome.get_gene_dict(gene, spec))
        return gdicts

    @staticmethod
    def expandLinks(parent_link, uniq_parent_name, flat_links, exp_links):
        children = [l for l in flat_links if l.parent_name == parent_link.name]
        sibling_ind = 1
        for c in children:
            for r in range(int(c.recur)):
                sibling_ind  = sibling_ind +1
                c_copy = copy.copy(c)
                c_copy.parent_name = uniq_parent_name
                uniq_name = c_copy.name + str(len(exp_links))
                #print("exp: ", c.name, " -> ", uniq_name)
                c_copy.name = uniq_name
                c_copy.sibling_ind = sibling_ind
                exp_links.append(c_copy)
                assert c.parent_name != c.name, "Genome::expandLinks: link joined to itself: " + c.name + " joins " + c.parent_name 
                Genome.expandLinks(c, uniq_name, flat_links, exp_links)

    @staticmethod#Touched this
    def genome_to_links(gdicts,fixed_shape = None): #for extension, <0.5 for cylinder, and >=0.5 for box
        links = []
        spine_link_names = ['spine0','spine1','spine2']
        #create the base link , the first spine
        base_link = URDFLink(name=spine_link_names[0],parent_name="None",recur = 1, link_length=0.5, link_radius=0.2,link_shape=0.5)# cuboid shaped
        links.append(base_link)

        #the rest of the spine
        for i in range(1, len(spine_link_names)):
            spine_link = URDFLink(
                name=spine_link_names[i], 
                parent_name=spine_link_names[i-1], # Attach to previous spine link
                recur=1, 
                link_length=0.5, 
                link_radius=0.15,
                link_shape=0.2, # cylinders
                joint_origin_xyz_3=0.5, # move it along the Z-axis of the parent
                joint_axis_xyz=0.2 # pivot on the X-axis
            )
            links.append(spine_link)

        # to create symmetrical limb pairs
        limb_pair_ind = 0
        for gdict in gdicts:
            # determine which spine link to attach to
            parent_ind = int(gdict["joint-parent-spine-link"] * len(spine_link_names)) #--> final_index = int(scaled_value * 3)
            parent_name = spine_link_names[parent_ind]

            link_shape_to_use = gdict["link-shape"]# To enable change of shape or a fixed shape
            if fixed_shape is not None:
                link_shape_to_use = fixed_shape

            # create left limb
            left_limb = URDFLink(
                name=f"left_limb_{limb_pair_ind}",
                parent_name=parent_name,
                recur=gdict["link-recurrence"] + 1,

                #symmetrical placement along the Y-axis
                joint_origin_xyz_2=gdict["joint-y-offset"], #positive Y

                link_length=gdict["link-length"], 
                link_radius=gdict["link-radius"], 
                link_mass=gdict["link-mass"],
                joint_type=gdict["joint-type"],
                #joint_parent=gdict["joint-parent"], #now irrelevant
                joint_axis_xyz=gdict["joint-axis-xyz"],
                joint_origin_rpy_1=gdict["joint-origin-rpy-1"],
                joint_origin_rpy_2=gdict["joint-origin-rpy-2"],
                joint_origin_rpy_3=gdict["joint-origin-rpy-3"],
                joint_origin_xyz_1=gdict["joint-origin-xyz-1"],
                joint_origin_xyz_3=gdict["joint-origin-xyz-3"],
                control_waveform=gdict["control-waveform"],
                control_amp=gdict["control-amp"],
                control_freq=gdict["control-freq"]
                ,link_shape=link_shape_to_use #to now change shape
            )
            links.append(left_limb)

            #ccopy of the left, but with mirrored y-offset and a phase-shifted motor
            right_limb = copy.copy(left_limb) # Start with a copy
            right_limb.name = f"right_limb_{limb_pair_ind}"
            right_limb.joint_origin_xyz_2 = -gdict["joint-y-offset"] # Mirrored Y (Negative)
            

            #for now, created an identical motor.
            links.append(right_limb)
            
            limb_pair_ind += 1
            
        return links


    @staticmethod
    def crossover(g1, g2):
        x1 = random.randint(0, len(g1)-1)
        x2 = random.randint(0, len(g2)-1)
        g3 = np.concatenate((g1[x1:], g2[x2:])) 
        if len(g3) > len(g1):
            g3 = g3[0:len(g1)] 
        return g3

    @staticmethod
    def point_mutate(genome, rate, amount):
        new_genome = copy.copy(genome)
        for gene in new_genome:
            for i in range(len(gene)):
                if random.random() < rate:
                    gene[i] += 0.1
                if gene[i] >= 1.0:
                    gene[i] = 0.9999
                if gene[i] < 0.0:
                    gene[i] = 0.0
        return new_genome

    @staticmethod
    def shrink_mutate(genome, rate):
        if len(genome) == 1:
            return copy.copy(genome)
        if random.random() < rate:
            ind = random.randint(0, len(genome)-1)
            new_genome = np.delete(genome, ind, 0)
            return new_genome
        else:
            return copy.copy(genome)

    @staticmethod
    def grow_mutate(genome, rate):
        if random.random() < rate:
            gene = Genome.get_random_gene(len(genome[0]))
            new_genome = copy.copy(genome)
            new_genome = np.append(new_genome, [gene], axis=0)
            return new_genome
        else:
            return copy.copy(genome)


    @staticmethod
    def to_csv(dna, csv_file):
        csv_str = ""
        for gene in dna:
            for val in gene:
                csv_str = csv_str + str(val) + ","
            csv_str = csv_str + '\n'

        with open(csv_file, 'w') as f:
            f.write(csv_str)

    @staticmethod
    def from_csv(filename):
        csv_str = ''
        with open(filename) as f:
            csv_str = f.read()   
        dna = []
        lines = csv_str.split('\n')
        for line in lines:
            vals = line.split(',')
            gene = [float(v) for v in vals if v != '']
            if len(gene) > 0:
                dna.append(gene)
        return dna

class URDFLink:
    def __init__(self, name, parent_name, recur, 
                link_length=0.1, 
                link_radius=0.1, 
                link_mass=0.1,
                joint_type=0.1,
                joint_parent=0.1,
                joint_axis_xyz=0.1,
                joint_origin_rpy_1=0.1,
                joint_origin_rpy_2=0.1,
                joint_origin_rpy_3=0.1,
                joint_origin_xyz_1=0.1,
                joint_origin_xyz_2=0.1,
                joint_origin_xyz_3=0.1,
                control_waveform=0.1,
                control_amp=0.1,
                control_freq=0.1,
                link_shape = 0.1 #to now change shape#Touched this
                ):
        self.name = name
        self.parent_name = parent_name
        self.recur = recur 
        self.link_length=link_length 
        self.link_radius=link_radius
        self.link_mass=link_mass
        self.joint_type=joint_type
        self.joint_parent=joint_parent
        self.joint_axis_xyz=joint_axis_xyz
        self.joint_origin_rpy_1=joint_origin_rpy_1
        self.joint_origin_rpy_2=joint_origin_rpy_2
        self.joint_origin_rpy_3=joint_origin_rpy_3
        self.joint_origin_xyz_1=joint_origin_xyz_1
        self.joint_origin_xyz_2=joint_origin_xyz_2
        self.joint_origin_xyz_3=joint_origin_xyz_3
        self.control_waveform=control_waveform
        self.control_amp=control_amp
        self.control_freq=control_freq
        self.link_shape=link_shape  #to now change shape#Touched this
        self.sibling_ind = 1

    def to_link_element(self, adom):
        link_tag = adom.createElement("link")
        link_tag.setAttribute("name", self.name)

        vis_tag = adom.createElement("visual")
        geom_tag = adom.createElement("geometry")

        #to now change shape#Touched this
        shape_val = self.link_shape
        if shape_val < 0.5:
            # Cylinder
            shape_tag = adom.createElement("cylinder")
            shape_tag.setAttribute("length", str(self.link_length))
            shape_tag.setAttribute("radius", str(self.link_radius))
        elif shape_val >= 0.5:
            # Box
            shape_tag = adom.createElement("box")
            shape_tag.setAttribute("size", f"{self.link_radius*2} {self.link_radius*2} {self.link_length}")

        geom_tag.appendChild(shape_tag)
        vis_tag.appendChild(geom_tag)
        link_tag.appendChild(vis_tag)
        
        coll_tag = adom.createElement("collision")
        c_geom_tag = adom.createElement("geometry")

        #to now change shape's collision#Touched this
        if shape_val < 0.5:
            c_shape_tag = adom.createElement("cylinder")
            c_shape_tag.setAttribute("length", str(self.link_length))
            c_shape_tag.setAttribute("radius", str(self.link_radius))
        elif shape_val >= 0.5:
            c_shape_tag = adom.createElement("box")
            c_shape_tag.setAttribute("size", f"{self.link_radius*2} {self.link_radius*2} {self.link_length}")


        c_geom_tag.appendChild(c_shape_tag)
        coll_tag.appendChild(c_geom_tag)
        link_tag.appendChild(coll_tag)

        inertial_tag = adom.createElement("inertial")
        mass_tag = adom.createElement("mass")


        mass = 0.1
        #different shape, different volume#Touched this
        if shape_val < 0.5:
            mass = np.pi * (self.link_radius * self.link_radius) * self.link_length 
        elif shape_val >=  0.5:
            mass = (self.link_radius * 2) * (self.link_radius * 2) * self.link_length # radius x 2 = length/diameter

        
        mass_tag.setAttribute("value", str(mass))
        inertia_tag = adom.createElement("inertia")

        inertia_tag.setAttribute("ixx", "0.03")
        inertia_tag.setAttribute("iyy", "0.03")
        inertia_tag.setAttribute("izz", "0.03")
        inertia_tag.setAttribute("ixy", "0")
        inertia_tag.setAttribute("ixz", "0")
        inertia_tag.setAttribute("iyx", "0")
        inertia_tag.setAttribute("iyz", "0")#Touched this
        inertial_tag.appendChild(mass_tag)
        inertial_tag.appendChild(inertia_tag)
        

        link_tag.appendChild(vis_tag)
        link_tag.appendChild(coll_tag)
        link_tag.appendChild(inertial_tag)


        
        return link_tag

    def to_joint_element(self, adom):
        joint_tag = adom.createElement("joint")
        joint_tag.setAttribute("name", self.name + "_to_" + self.parent_name)
        if self.joint_type >= 0.5:
            joint_tag.setAttribute("type", "revolute")
        else:
            joint_tag.setAttribute("type", "revolute")
        parent_tag = adom.createElement("parent")
        parent_tag.setAttribute("link", self.parent_name)
        child_tag = adom.createElement("child")
        child_tag.setAttribute("link", self.name)
        axis_tag = adom.createElement("axis")

        if self.joint_axis_xyz <= 0.33:
            axis_tag.setAttribute("xyz", "1 0 0")
        if self.joint_axis_xyz > 0.33 and self.joint_axis_xyz <= 0.66:
            axis_tag.setAttribute("xyz", "0 1 0")
        if self.joint_axis_xyz > 0.66:
            axis_tag.setAttribute("xyz", "0 0 1")


        limit_tag = adom.createElement("limit")

        limit_tag.setAttribute("effort", "300") #CHANGED FROM 1. Final 300#Touched this
        limit_tag.setAttribute("upper", "-3.1415")
        limit_tag.setAttribute("lower", "3.1415")
        limit_tag.setAttribute("velocity", "1")

        orig_tag = adom.createElement("origin")

        #define limbs mirrored limbs postion#Touched this
        rpy1 = self.joint_origin_rpy_1
        rpy2 = self.joint_origin_rpy_2
        rpy3 = self.joint_origin_rpy_3

        if "right_limb" in self.name:
            #mirror across the XZ plane, we invert the Roll (X-rot) and Yaw (Z-rot)
            #pitch (Y-rot) remains the same.
            rpy1 = -rpy1 #invert Roll
            rpy3 = -rpy3 #invert Yaw
        rpy = str(rpy1) + " " + str(rpy2) + " " + str(rpy3)

        orig_tag.setAttribute("rpy", rpy)

        xyz = str(self.joint_origin_xyz_1) + " " + str(self.joint_origin_xyz_2) + " " + str(self.joint_origin_xyz_3)
        orig_tag.setAttribute("xyz", xyz)

        joint_tag.appendChild(parent_tag)
        joint_tag.appendChild(child_tag)
        joint_tag.appendChild(axis_tag)
        joint_tag.appendChild(limit_tag)
        joint_tag.appendChild(orig_tag)
        return joint_tag
            



