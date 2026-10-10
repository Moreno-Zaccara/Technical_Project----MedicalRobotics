import sys
import Sofa
import Sofa.Core
import Sofa.Simulation
import csv
import Sofa.constants.Key as Key
import os
import numpy as np

import matplotlib.pyplot as plt

#definizione costanti
Straight_length_1 = 115
Straight_length_2 = 140
Straight_length_3 = 175
Curved_length_1 = 40
Curved_length_2 = 50
Curved_length_3 = 65
Tube_radius_1 = 1.0
Tube_radius_2 = 0.85
Tube_radius_3 = 0.70
Radius_curvature_1 = 105
Radius_curvature_2 = 115
Radius_curvature_3 = 135


# 1. CONTROLLER HAPTICO + GESTIONE "RISCALDAMENTO"
class GeomagicCTRController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs) :   
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.mri = kwargs.get("mri")
        self.driver = kwargs.get("driver")
        self.lcp_ff = kwargs.get("lcp_ff") # Riferimento al componente forza
        self.name = "GeomagicCTRController"
        # Logica Tubi
        self.selected_tube = 0
        self.num_tubes = 3
        self.prev_btn1 = 0
        # Logica Movimento
        self.prev_pos_x = 0.0
        self.prev_pos_z = 0.0
        self.initialized = False
        # Logica Riscaldamento (Warm-up)
        self.step_count = 0
        self.warmup_steps = 100 # Aspetta circa 2-3 secondi

        self.target_force_coef = 0.05 # La forza finale che vogliamo
        # Impostiamo forza a 0 all'inizio per evitare esplosioni
        self.lcp_ff.forceCoef.value = 0.0
        # Sensibilità
        self.scale_insertion = 2.0
        self.scale_rotation = 0.2
        print(">>> INIZIALIZZAZIONE... ATTENDERE 2 SECONDI <<<")

    def onAnimateBeginEvent(self, event):
        # ••• FASE 1: RISCALDAMENTO --.
        if self.step_count < self. warmup_steps:
            self.step_count += 1
            # Sincronizziamo solo la posizione "precedente" per evitare salti
            # ma NON muoviamo il robot e NON diamo forza
            device_pos = self.driver.positionDevice.value
            self.prev_pos_x = device_pos[0]
            self.prev_pos_z = device_pos[2]

            if self.step_count == self. warmup_steps:
                print (">>> SISTEMA PRONTO: APTICA ATTIVATA <<<")
                self.lcp_ff.forceCoef.value = self.target_force_coef
            return

        #••• FASE 2: CONTROLLO NORMALE •••

        # Leggi posizione
        device_pos = self.driver.positionDevice.value
        curr_x = device_pos[0]
        curr_z = device_pos[2]
        # Leggi tasti
        btn_grey = self.driver.button1.value
        btn_white = self.driver.button2.value
        # Cambio Tubo
        if btn_grey == 1 and self.prev_btn1 == 0:
            self.selected_tube = (self.selected_tube+1) % self.num_tubes
            print(f"---> TUBO SELEZIONATO: {self.selected_tube+1}")
        self.prev_btn1 = btn_grey
        # Frizione (Tasto Bianco)
        if btn_white == 1:
            self.prev_pos_x = curr_x
            self.prev_pos_z = curr_z
            return

        # Calcolo Delta
        delta_x = curr_x - self.prev_pos_x
        delta_z = curr_z - self.prev_pos_z

        # Deadzone
        if abs (delta_x)< 0.002: delta_x = 0
        if abs (delta_z)< 0.002: delta_z = 0
        # Applica alRobot
        current_xtip = list(self.mri.xtip.value)
        current_rot = list(self.mri.rotationInstrument.value)
        current_xtip[self.selected_tube] += (delta_z * self.scale_insertion) * -1.0
        current_rot[self.selected_tube] += (delta_x*self.scale_rotation)
        self.mri.xtip.value = np.array(current_xtip)
        self.mri.rotationInstrument.value = np.array(current_rot)
        # Aggiorna storia
        self.prev_pos_x = curr_x
        self.prev_pos_z = curr_z

class KeyBoardController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.name = 'KeyBoardController'
        self.ir_controller = kwargs.get('irController')
        self.rootNode = kwargs.get('rootNode')

        self.tube = 1    # (1 = TUBE_1, 2 = TUBE_2, 3 = TUBE_3)
        self.translation_step = 1.0 # [mm]
        self.rotation_step = np.radians(22.5) # [rad]
        # Loggig data
        self.variazione_percentuale = 13.0 # in valore assoluto
        self.print_interval = 1
        self.last_print_time = 0.0   
        # --- ONLINE FORCE PLOT ---
        plt.ion()  # modalità interattiva
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Forza vs Tempo")
        self.ax.set_xlabel("Tempo [s]")
        self.ax.set_ylabel("Forza [N]")
        
        self.times = []
        self.forces = []
        
        self.line, = self.ax.plot(self.times, self.forces, '-b')
        self.fig.show()
        self.fig.canvas.draw()

        print("BENVENUTO")
        print("Selezionare il tubo da controllare (Ctrl + 0/1/2)")
        print("Successivamente selezionare il tipo di moto: traslazione in avanti o indietro (Ctrl + arrow up/down) - rotazione in verso orario o antiorario (Ctrl + arrow left /right)")

    def onKeypressedEvent(self, c):
        key = c['key']
        if key == "0":
            print("Selezionato il tubo 1")
            self.tube = 1
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == "1":
            print("Selezionato il tubo 2")
            self.tube = 2
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == "2":
            print("Selezionato il tubo 3")
            self.tube = 3
            self.ir_controller.controlledInstrument.value = self.tube -1
        elif key == Key.downarrow:
            print("Scelta traslazione all'indietro")
            #self.action = 'j'
            self.translate(self.tube, -1.0)
        elif key == Key.uparrow:
            print("Scelta traslazione in avanti")
            #self.action = 'k'
            self.translate(self.tube, 1.0)
        elif key == Key.leftarrow:
            print("Scelta rotazione oraria")
            #self.action = 'm'
            self.rotate(self.tube, -0.4)
        elif key == Key.rightarrow:
            print("Scelta rotazione antioraria")
            #self.action = 'n'
            self.rotate(self.tube, 0.4)
        elif key == "W":
            Sofa.Helper.msg_info("KeyBoardController", "Generazione workspace in condizioni nominali...")
            self.generate_workspace("workspace_nominal.csv", percentage=100.0)
        elif key == "Q":
            self.variazione_percentuale *= -1
            Sofa.Helper.msg_info("KeyBoardController", f"Generazione workspace con variazione del -13%")
            self.generate_workspace("workspace_var_-13.csv", percentage=87.0)
        elif key == "E":
            Sofa.Helper.msg_info("KeyBoardController", f"Generazione workspace con variazione del +13%")
            self.generate_workspace("workspace_var_+13.csv", percentage=113.0)


    #IMPLEMENTAZIONE CONTROLLO VERO E PROPRIO

    def translate(self, tube, quantity):
        with self.ir_controller.xtip.writeable() as d: d[tube-1] = d[tube-1] + quantity
    def rotate(self, tube, quantity):
        with self.ir_controller.rotationInstrument.writeable() as d: d[tube-1] = d[tube-1]+ quantity

    def update_radius_curvature(self, percentage=100.0):
        percentage /= 100.0
        # Nominal values
        r1 = Radius_curvature_1
        r2 = Radius_curvature_2
        r3 = Radius_curvature_3
        # Update radius
        #r1 *= percentage
        r2 *= percentage
        r3 *= percentage
        # Update the sofa diameter values
        # self.rootNode.TUBE_1.SpireSection.spireDiameter.value = 2*r1
        self.rootNode.TUBE_2.SpireSection.spireDiameter.value = 2*r2
        self.rootNode.TUBE_3.SpireSection.spireDiameter.value = 2*r3
        # Reset rest shape
        # self.rootNode.TUBE_1.RestShape_1.reinit()
        self.rootNode.TUBE_2.RestShape_2.reinit()
        self.rootNode.TUBE_3.RestShape_3.reinit()

    def generate_workspace(self, filename, percentage):
        self.update_radius_curvature(percentage=percentage)

        workspace_points = []

        # passo di inserimento in mm
        insertion_step = 40     
        # passo di rotazione in gradi
        rotation_step = 45      

        LenIns_1 = (Straight_length_1 + Curved_length_1)
        LenIns_2 = (Straight_length_2 + Curved_length_2)
        LenIns_3 = (Straight_length_3 + Curved_length_3)

        insertion_range_1 = range(0, int(LenIns_1) + 1, insertion_step)
        rotation_range_1  = range(-90, 91, rotation_step)

        insertion_range_2 = range(0, int(LenIns_2) + 1, insertion_step)
        rotation_range_2  = range(-90, 91, rotation_step)

        insertion_range_3 = range(0, int(LenIns_3) + 1, insertion_step)	
        rotation_range_3  = range(-90, 91, rotation_step)

        i=1

        n_ins1 = len(insertion_range_1)
        n_rot1 = len(rotation_range_1)
        n_ins2 = len(insertion_range_2)
        n_rot2 = len(rotation_range_2)
        n_ins3 = len(insertion_range_3)
        n_rot3 = len(rotation_range_3)

        total_points = n_ins1 * n_rot1 * n_ins2 * n_rot2 * n_ins3 * n_rot3	

        for ins1 in insertion_range_1:
            for rot1 in rotation_range_1:
                rot1 = np.radians(rot1)
                for ins2 in insertion_range_2:
                    for rot2 in rotation_range_2:
                        rot2 = np.radians(rot2)
                        for ins3 in insertion_range_3:
                            for rot3 in rotation_range_3:
                                rot3 = np.radians(rot3)

                                # Sofa.Helper.msg_info("KeyBoardController", f"Generazione punto {i} / {total_points} \r")
                                print(f"[KeyBoardController] Generazione punto {i} / {total_points}", end="\r")
                            
                                with self.ir_controller.xtip.writeable() as d:
                                    d[0] = ins1
                                    d[1] = ins2
                                    d[2] = ins3
                                with self.ir_controller.rotationInstrument.writeable() as d:
                                    d[0] = rot1
                                    d[1] = rot2
                                    d[2] = rot3
                                i+=1
                           
                                Sofa.Simulation.animate(self.rootNode, self.rootNode.dt.value)	
                            
                                tip_pose = self.rootNode.CTR.DOFs.position.value[-1]
                                tip_position = tip_pose[:3]
                            
                                workspace_points.append([
                                    tip_position[0], tip_position[1], tip_position[2], ins3
                                ])
        # Alla fine, salva sul file
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow([
                "X", "Y", "Z", "Ins3"
            ])
            writer.writerows(workspace_points)

        Sofa.Helper.msg_info("KeyBoardController", f"Workspace salvato correttamente in {filename}")

    
    def onAnimateEndEvent(self, event):
         self.get_force_value()
        
    def get_force_value(self):
        t = self.rootNode.time.value    
        solver = self.rootNode.getObject("solver")
        forces_vector = solver.constraintForces.value
        scale_factor = 1/100000   # <-- Fattore di scala

        if forces_vector is not None and forces_vector.size > 0 and (t - self.last_print_time) > self.print_interval:
            force_value = np.linalg.norm(forces_vector)
            Sofa.Helper.msg_info("KeyBoardController", f"\nSensore di Forza: F = {force_value:.3f} N\n")
            self.last_print_time = t

             # --- ONLINE PLOT UPDATE ---
            self.times.append(t)
            self.forces.append(force_value * scale_factor)

            # Aggiorna la curva
            self.line.set_xdata(self.times)
            self.line.set_ydata(self.forces)

            # Aggiorna i limiti
            self.ax.relim()
            self.ax.autoscale_view()

            # Refresh del grafico
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        


MESH_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mesh')

def create_torus(rootNode):
    torus = rootNode.addChild('torus')
    torus.addObject('EulerImplicitSolver', rayleighStiffness="0.1", rayleighMass="0.1")	
    torus.addObject('SparseLDLSolver')
    torus.addObject('MeshOBJLoader', name="torusLoader", filename=os.path.join(MESH_PATH, 'torus.obj'), translation="20 -15 80", rotation="45 0 0", scale="3")	# caricamento mesh di partenza
    torus.addObject('MeshGenerationFromPolyhedron', name="tetraGenerator", inputPoints="@torusLoader.position", inputTriangles="@torusLoader.triangles", inputQuads="@torusLoader.quads", drawTetras="0", facetSize="5", facetApproximation="1", cellRatio="2", cellSize="5")	# creazione mesh volumetrica
    torus.addObject('MechanicalObject', name="dofs", position="@tetraGenerator.outputPoints")	
    torus.addObject('TetrahedronSetTopologyContainer', name="topo", tetrahedra="@tetraGenerator.outputTetras")
    torus.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    torus.addObject('ParallelTetrahedronFEMForceField', name="FEM", youngModulus="5e4", poissonRatio="0.45", method="large") 
    torus.addObject('UniformMass', name="mass", totalMass="100.0")
    torus.addObject('LinearSolverConstraintCorrection')
    torusVisu = torus.addChild('Visu')											      
    torusVisu.addObject('OglModel', name="Visual", src="@../torusLoader", color="0.7 0.3 0.1")
    torusVisu.addObject('BarycentricMapping', input="@..", output="@Visual")
    torusCollis = torus.addChild('Collision')										    
    torusCollis.addObject('MeshTopology', src="@../torusLoader")
    torusCollis.addObject('MechanicalObject')
    torusCollis.addObject('TriangleCollisionModel')
    torusCollis.addObject('LineCollisionModel')
    torusCollis.addObject('PointCollisionModel')
    torusCollis.addObject('BarycentricMapping', input="@..", output="@.")
    constraintNode = torus.addChild('Constraints')								
    constraintNode.addObject('BoxROI', name="fixedBox", box="20 -15 80  55 -5 40", drawBoxes="1", position="@../dofs.rest_position")
    constraintNode.addObject('BoxROI', name="fixedBox", box="-55 -15 80  -20 -5 40", drawBoxes="1", position="@../dofs.rest_position")
    constraintNode.addObject('FixedProjectiveConstraint', indices="@fixedBox.indices")
    Sofa.Helper.msg_info("createScene", "Torus creato correttamente")

def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='Sofa.Component.AnimationLoop Sofa.Component.SolidMechanics.Spring MultiThreading Sofa.Component.Topology.Mapping Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Collision.Response.Contact Sofa.Component.Collision.Geometry Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Detection.Algorithm BeamAdapter Sofa.Component.Constraint.Projective Sofa.Component.LinearSolver.Direct Sofa.Component.ODESolver.Backward Sofa.Component.StateContainer Sofa.Component.Topology.Container.Constant Sofa.Component.Topology.Container.Grid Sofa.Component.Visual Sofa.Component.Mapping.Linear Sofa.Component.Topology.Container.Dynamic Sofa.GL.Component.Rendering3D CGALPlugin Sofa.Component.IO.Mesh Sofa.Component.Engine.Select Sofa.Component.Mass Geomagic')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.gravity = [0, 0, 0]
    rootNode.dt = 0.05

    # Create torus for the final part   
    create_torus(rootNode=rootNode)

    # collision pipeline

    solver = rootNode.addObject('GenericConstraintSolver', name='solver', computeConstraintForces="1", tolerance='1e-6', maxIterations='1000')
    rootNode.addObject('CollisionPipeline', verbose='0')
    rootNode.addObject('ParallelBruteForceBroadPhase')
    rootNode.addObject('ParallelBVHNarrowPhase')
    rootNode.addObject('CollisionResponse', name='response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', name='proximity', alarmDistance='2', contactDistance='0.5', angleCone='0.0')

    # TUBO 1

    TUBE_1 = rootNode.addChild('TUBE_1', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_1.addObject('RodStraightSection', name='StraightSection', length = Straight_length_1, radius = Tube_radius_1, youngModulus=1e4, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80, poissonRatio=0.3)
    TUBE_1.addObject('RodSpireSection', name='SpireSection', length = Curved_length_1, spireDiameter = 2*Radius_curvature_1, spireHeight=0.0, youngModulus=1e4, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80,poissonRatio=0.3)
    TUBE_1.addObject('WireRestShape', template='Rigid3d', name='RestShape_1', wireMaterials='@StraightSection @SpireSection')

    #   -proprieta meccaniche

    TUBE_1.addObject('EdgeSetTopologyContainer', name='meshLines_1')
    TUBE_1.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_1.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_1.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_1')

    # TUBO 2

    TUBE_2 = rootNode.addChild('TUBE_2', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_2.addObject('RodStraightSection', name='StraightSection', length = Straight_length_2, radius = Tube_radius_2, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80,poissonRatio=0.3)
    TUBE_2.addObject('RodSpireSection', name='SpireSection', length = Curved_length_2, spireDiameter = 2*Radius_curvature_2, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80,poissonRatio=0.3)
    TUBE_2.addObject('WireRestShape', template='Rigid3d', name='RestShape_2', wireMaterials='@StraightSection @SpireSection')

    #   -proprieta meccaniche

    TUBE_2.addObject('EdgeSetTopologyContainer', name='meshLines_2')
    TUBE_2.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_2.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_2.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_2')

    # TUBO 3

    TUBE_3 = rootNode.addChild('TUBE_3', bbox='-3 -6 -3 3 3 3')

    #   -sezioni e unione

    TUBE_3.addObject('RodStraightSection', name='StraightSection', length = Straight_length_3, radius = Tube_radius_3, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80,poissonRatio=0.3)
    TUBE_3.addObject('RodSpireSection', name='SpireSection', length = Curved_length_3, spireDiameter = 2*Radius_curvature_3, spireHeight=0.0, youngModulus=1e9, massDensity=1.55e-6, nbBeams=40, nbEdgesCollis=40, nbEdgesVisu=80,poissonRatio=0.3)
    TUBE_3.addObject('WireRestShape', template='Rigid3d', name='RestShape_3', wireMaterials='@StraightSection @SpireSection')

    #   -proprieta meccaniche

    TUBE_3.addObject('EdgeSetTopologyContainer', name='meshLines_3')
    TUBE_3.addObject('EdgeSetTopologyModifier', name='Modifier')
    TUBE_3.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    TUBE_3.addObject('MechanicalObject', template='Rigid3d', name='dofTopo_3')

     # Instrument Combined
    CTR = rootNode.addChild('CTR')
    CTR.addObject('EulerImplicitSolver', rayleighStiffness='0.2', rayleighMass='0.1', printLog='false')
    CTR.addObject('BTDLinearSolver')
    CTR.addObject('RegularGridTopology', name='meshLinesCombined', nx='241', ny='1', nz='1', xmin='0.0', xmax='1.0', ymin='0', ymax='0', zmin='1', zmax='1')
    CTR.addObject('MechanicalObject', template='Rigid3d', name='DOFs', showIndices='0', ry='-90')
    CTR.addObject('WireBeamInterpolation', name='Interpol_1', WireRestShape='@../TUBE_1/RestShape_1')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube1ForceField', interpolation='@Interpol_1')
    CTR.addObject('WireBeamInterpolation', name='Interpol_2', WireRestShape='@../TUBE_2/RestShape_2')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube2ForceField', interpolation='@Interpol_2')
    CTR.addObject('WireBeamInterpolation', name='Interpol_3', WireRestShape='@../TUBE_3/RestShape_3')
    CTR.addObject('AdaptiveBeamForceFieldAndMass', name='Tube3ForceField', interpolation='@Interpol_3')
    CTR.addObject('InterventionalRadiologyController', template="Rigid3d", name="IRController", instruments="Interpol_1 Interpol_2 Interpol_3", xtip="1 0 0", step="3", rotationInstrument="0 0 0", controlledInstrument="0", startingPos="0 0 0 0 0 0 0.")
    CTR.addObject('RestShapeSpringsForceField', points="@IRController.indexFirstNode", stiffness="1e8", angularStiffness="1e8")
    CTR.addObject('FixedProjectiveConstraint', name='FixedConstraint', indices='0')
    CTR.addObject('LinearSolverConstraintCorrection', wire_optimization='true')
    CTR.addObject("LCPForceFeedback", name="LCPFF1", activate=True, forceCoef=0.0001)

    #   -collisioni

    collis = CTR.addChild('Collis')
    collis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    collis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    collis.addObject('MechanicalObject', name='MechanicalObject', template='Vec3d')
    collis.addObject('MultiAdaptiveBeamMapping', controller='../IRController')
    collis.addObject('LineCollisionModel')
    collis.addObject('PointCollisionModel')

    #   -proprieta visuali 1

    visu_1 = CTR.addChild('visu_1', activated='true')
    visu_1.addObject('MechanicalObject', name='Quads')
    visu_1.addObject('QuadSetTopologyContainer', name='Container_1')
    visu_1.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_1.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_1.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='2', input='@../../TUBE_1/meshLines_1', output='@Container_1', flipNormals='true')
    visu_1.addObject('AdaptiveBeamMapping',  name='VisuMap_1', useCurvAbs='1', printLog='0', interpolation='@../Interpol_1', input='@../DOFs', output='@Quads')
    visuOgl_1 = visu_1.addChild('visuOgl_1', activated='true')
    visuOgl_1.addObject('OglModel', name='Visual', color='1 0 0', quads="@../Container_1.quads") #1 0 0 1
    visuOgl_1.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #   -proprieta visuali 2

    visu_2 = CTR.addChild('visu_2', activated='true')
    visu_2.addObject('MechanicalObject', name='Quads')
    visu_2.addObject('QuadSetTopologyContainer', name='Container_2')
    visu_2.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_2.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_2.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='1.5', input='@../../TUBE_2/meshLines_2', output='@Container_2', flipNormals='true')
    visu_2.addObject('AdaptiveBeamMapping',  name='VisuMap_2', useCurvAbs='1', printLog='0', interpolation='@../Interpol_2', input='@../DOFs', output='@Quads')
    visuOgl_2 = visu_2.addChild('visuOgl_2', activated='true')
    visuOgl_2.addObject('OglModel', name='Visual', color='0 1 0', quads="@../Container_2.quads") #0 1 0 1
    visuOgl_2.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #   -proprieta visuali 3

    visu_3 = CTR.addChild('visu_3', activated='true')
    visu_3.addObject('MechanicalObject', name='Quads')
    visu_3.addObject('QuadSetTopologyContainer', name='Container_3')
    visu_3.addObject('QuadSetTopologyModifier', name='Modifier')
    visu_3.addObject('QuadSetGeometryAlgorithms', name='GeomAlgo', template='Vec3d')
    visu_3.addObject('Edge2QuadTopologicalMapping', nbPointsOnEachCircle='10', radius='1', input='@../../TUBE_3/meshLines_3', output='@Container_3', flipNormals='true')
    visu_3.addObject('AdaptiveBeamMapping',  name='VisuMap_3', useCurvAbs='1', printLog='0', interpolation='@../Interpol_3', input='@../DOFs', output='@Quads')
    visuOgl_3 = visu_3.addChild('visuOgl_3', activated='true')
    visuOgl_3.addObject('OglModel', name='Visual', color='0 0 1', quads="@../Container_3.quads")
    visuOgl_3.addObject('IdentityMapping', input="@../Quads", output="@Visual")

    #aggiunta controllore alla scena

    rootNode.addObject(KeyBoardController(
        name="KeyBoardController",
        irController=CTR.getObject('IRController'),
        rootNode=rootNode
    ))

    # Controller
    # driver = rootNode.addChild("GeomagicNode")
    # driver.addObject("GeomagicDriver",
    #            name="GeomagicDriver",
    #            deviceName="Default Device",
    #            scale=1,
    #            positionBase="0 0 0",
    #            orientationBase="0 0 0 1",
    #            drawDevice=0,
    #            drawDeviceFrame=0,
    #            forceFeedBack="@CTR/LCPFF1",
    #            manualStart=False)
    #         #    drawDevice=1)
    # driver.addObject("MechanicalObject", name="geomagicState", template="Rigid3")

    # omni = rootNode.addChild("Omni")
    # omni.addObject("MechanicalObject",
    #             name="DOFs",
    #             template="Rigid3",
    #             position="@GeomagicDevice.positionDevice")
    
    # rootNode.addObject(GeomagicCTRController(
    #     mri = CTR.getObject('IRController'),
    #     driver = driver.getObject('GeomagicDriver'),
    #     lcp_ff = CTR.getObject('LCPFF1')
    # ))

    return rootNode;


def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)
    Sofa.Gui.GUIManager.Init("volumetric_snake", "qglviewer")
    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()