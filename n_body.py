import time
import pygame
import math
import numpy as np
from bodies import body, planet, star,particle

class simulation:
    def __init__(self):
        self.WIDTH =  800
        self.HEIGHT = 750
        self.FPS = 60
        self.G = 6.6743e-11  # Gravitational constant
        self.SCALE = 2e9 # Scale factor for positions
        self.bodies = []
        self.TIME = 24 * 60 * 60  #1 day initially
        self.DAY = 24 * 60 * 60  # Seconds in a day
        self.TIME_PASSED = 0
        self.SLIDER_WIDTH = 300
        self.SLIDER_HEIGHT = 10
        self.focus_index  = -1
        self.camera_position = np.array([0, 0])  # Camera position

        self.simulation_running = False

        self.preview_data = None
        self.dummy_body = None
        self.current_mode = "star"
        self.preview_mode = False

        # Add object mode: "star" or "planet"
        self.current_mode = "star"
        self.min_mass_pluto = 1.31e22
        self.max_mass_jupiter = 1.90e27
        self.PROXIMA_MIN_MASS = 2.38e29


        self.dragging_slider = False  # Slider dragging state

        pygame.init()
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Star and Planet Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 16)


    def add_body(self, position, velocity, mass=None,status=None):
        if status == "star":
            mass if mass else self.PROXIMA_MIN_MASS
            new_body = star(position, velocity, mass)
        elif status == "planet":
            mass = min(self.max_mass_jupiter, mass)
            new_body = planet(position, velocity, mass)
        else:
            new_body = particle(position,velocity,mass)
        self.bodies.append(new_body)
        #change focus to recebtly added body if there are no bodies
        if self.focus_index == -1:
            self.focus_index = 0
        #self.focus_index = len(self.bodies) - 1
    
    def compute_forces(self):
        forces = np.zeros((len(self.bodies), 2), dtype=np.float64)
        positions = np.array([body.position for body in self.bodies], dtype=np.float64)
        masses = np.array([body.mass for body in self.bodies], dtype=np.float64)
        epsilon = 1e-5  # Softening parameter

        for i, body in enumerate(self.bodies):
            r_vectors = positions - positions[i]  # Vector differences
            r_magnitudes = np.linalg.norm(r_vectors, axis=1)  # Distances
            r_magnitudes[i] = 1  # Avoid self-interaction
            softened_magnitudes = (r_magnitudes**2 + epsilon**2)**(3/2)  # Plummer softening
            forces[i] = np.sum(
                self.G * masses[i] * masses[:, None] * r_vectors / softened_magnitudes[:, None],
                axis=0,
            )
        return forces


    def update_positions(self):
        # Step 1: Compute initial forces
        forces = self.compute_forces()
        self.TIME_PASSED += self.TIME
        
        # Step 2: Half-step velocity update and full-step position update
        for i, body in enumerate(self.bodies):
            acceleration = forces[i] / body.mass
            body.velocity += 0.5 * acceleration * self.TIME
            body.position += body.velocity * self.TIME
        
        # Step 5: Recompute forces after positions are updated
        forces = self.compute_forces()
        
        # Step 6: Finalize velocity updates
        for i, body in enumerate(self.bodies):
            acceleration = forces[i] / body.mass
            body.velocity += 0.5 * acceleration * self.TIME

    def get_scaled_radius(self,body):
        radius = body.radius / self.SCALE
        return max(radius, 1)

    def get_position_in_simulation(self, screen_position):
        # Reverse the screen-to-simulation transformation
        simulation_position = np.array([
            (screen_position[0] - self.WIDTH / 2) * self.SCALE,
            -(screen_position[1] - self.HEIGHT / 2) * self.SCALE
        ]) + self.camera_position
        return simulation_position
    
    #basic calculations
    def calculate_orbital_velocity_x_y(self,mass1,mass2,distance):
        return math.sqrt(self.G * (mass1+mass2)/distance)
    
    def calculate_escape_velocity(self,mass1,mass2,distance):
        return math.sqrt(2 * self.G * (mass1+mass2)/distance)
    
    def load_particle_cloud(self):
        self.reset_bodies()  # Clear any existing bodies

        # Constants for particle initialization
        num_particles = 1000  # Number of particles
        mass_range = (1e20, 1e25)  # Mass range of particles in kilograms
        position_range = (-1e11, 1e11)  # Positions in meters
        velocity_range = (-1e3, 1e3)  # Initial velocities in m/s

        for _ in range(num_particles):
            mass = np.random.uniform(*mass_range)
            position = np.random.uniform(*position_range, size=2)  # 2D positions (x, y)
            velocity = np.random.uniform(*velocity_range, size=2)  # 2D velocities (vx, vy)
            self.add_body(position=position, velocity=velocity, mass=mass, status="particle")
        self.TIME = 1e4
        self.TIME_PASSED = 0
    
    def load_solar_system(self):
        self.reset_bodies()
        self.add_body(position=np.array([0, 0]), velocity=np.array([0, 0]), mass=1.989e30, status="star")  # Sun
        self.add_body(position=np.array([5.79e10, 0]), velocity=np.array([0, 47400]), mass=3.3011e23, status="planet")  # Mercury
        self.add_body(position=np.array([1.082e11, 0]), velocity=np.array([0, 35020]), mass=4.867e24, status="planet")  # Venus
        self.add_body(position=np.array([1.5e11, 0]), velocity=np.array([0, 30000]), mass=5.972e24, status="planet")  # Earth
        self.add_body(position=np.array([2.279e11, 0]), velocity=np.array([0, 24000]), mass=6.417e23, status="planet")  # Mars
        self.add_body(position=np.array([7.785e11, 0]), velocity=np.array([0, 13000]), mass=1.898e27, status="planet")  # Jupiter
        self.add_body(position=np.array([1.433e12, 0]), velocity=np.array([0, 9700]), mass=5.683e26, status="planet")  # Saturn
        self.add_body(position=np.array([2.87e12, 0]), velocity=np.array([0, 6800]), mass=8.681e25, status="planet")  # Uranus
        self.add_body(position=np.array([4.495e12, 0]), velocity=np.array([0, 5400]), mass=1.024e26, status="planet")  # Neptune
        self.add_body(position=np.array([5.906e12, 0]), velocity=np.array([0, 4700]), mass=1.303e22, status="planet")  # Pluto

    def initialize_alpha_centauri_system(self):
        # Constants
        self.reset_bodies()
        AU = 1.496e11  # meters in an AU
        YEAR = 365.25 * 24 * 3600  # seconds in a year
        SUN_MASS = 1.989e30  # kg
        EARTH_MASS = 5.972e24  # kg

        self.SCALE = 10*AU
        
        # Alpha Centauri A
        position_a = np.array([11.7 * AU, 0])  # Place A at periapsis
        velocity_a = np.array([0, 2840])  # Velocity in m/s  # Approximate orbital velocity at periapsis
        self.add_body(position_a, velocity_a, mass=1.1 * SUN_MASS, status="star")

        # Hypothetical planet around Alpha Centauri A
        position_pa = position_a + np.array([1 * AU, 0])  # 1 AU from Alpha Centauri A
        velocity_pa = velocity_a + np.array([0, 29800])  # Orbital velocity
        self.add_body(position_pa, velocity_pa, mass=EARTH_MASS, status="planet")

        # Alpha Centauri B
        position_b = np.array([-11.7 * AU, 0])  # Place B at apoapsis
        velocity_b = np.array([0, -3110])  # Opposite velocity direction
        self.add_body(position_b, velocity_b, mass=0.907 * SUN_MASS, status="star")

        # Proxima Centauri
        position_proxima = np.array([13_000 * AU, 0])  # Approx. distance from barycenter
        velocity_proxima = np.array([0, 294])  # Very slow orbital velocity
        self.add_body(position_proxima, velocity_proxima, mass=0.12 * SUN_MASS, status="star")

        # Hypothetical planet around Alpha Centauri B
        position_pb = position_b + np.array([0.5 * AU, 0])  # 0.5 AU from Alpha Centauri B
        velocity_pb = velocity_b + np.array([0, 35000])  # Orbital velocity
        self.add_body(position_pb, velocity_pb, mass=EARTH_MASS, status="planet")

        # Proxima b
        position_proxima_b = position_proxima + np.array([0.0485 * AU, 0])  # Semi-major axis
        velocity_proxima_b = velocity_proxima + np.array([0, 4.77e04])  # Orbital velocity
        self.add_body(position_proxima_b, velocity_proxima_b, mass=1.17 * EARTH_MASS, status="planet")

    def getTime(self):
        if self.TIME_PASSED < 365*self.DAY:
            return f"{self.TIME_PASSED/self.DAY:.2f} days"
        elif self.TIME_PASSED < 365*24*self.DAY:
            return f"{self.TIME_PASSED/(365*self.DAY):.2f} years"
        else:
            return f"{self.TIME_PASSED/(365*24*self.DAY):.2f} centuries"
        

    def draw_bodies(self):
        for body in self.bodies:
            # Get position and radius for drawing
            radius = self.get_scaled_radius(body)
            position = (body.position[:2] - self.camera_position) / self.SCALE  # Scaling to fit the screen
            color = body.render()['color']  # Getting the color from the render method

            # Convert the position to screen coordinates
            x = int(self.WIDTH / 2 + position[0])
            y = int(self.HEIGHT / 2 - position[1])

            # Draw the body (planet or star)
            # Ensure the object is drawn until the entire circle is out of the screen borders
            if x + radius > 0 and x - radius < self.WIDTH and y + radius > 0 and y - radius < self.HEIGHT:
                pygame.draw.circle(self.screen, color, (x, y), int(radius))

        
            # Optionally, add labels for bodies
            if self.bodies[self.focus_index] != body:
                if body.status == 'star':
                    label = self.font.render(f"Star: {body.mass:.2e} kg", True, (255, 255, 255))
                    self.screen.blit(label, (x + 10, y + 10))
                elif body.status == 'planet':
                    if self.SCALE < 5e10: # only show labels for planets for small scales
                        label = self.font.render(f"Planet: {body.mass:.2e} kg", True, (255, 255, 255))
                        self.screen.blit(label, (x + 10, y + 10))
                
            if self.focus_index != -1:
                mass_label = self.font.render(f"Mass: {self.bodies[self.focus_index].mass:.2e} kg", True, (255, 255, 255))
                velocity_label_x_and_y = self.font.render(f"Velocity: {self.bodies[self.focus_index].velocity[0]:.2e} m/s, {self.bodies[self.focus_index].velocity[1]:.2e} m/s", True, (255, 255, 255))
                type_label = self.font.render(f"Type: {self.bodies[self.focus_index].status}", True, (255, 255, 255))
                position_label = self.font.render(f"Position: {self.bodies[self.focus_index].position[0]:.2e} m, {self.bodies[self.focus_index].position[1]:.2e} m", True, (255, 255, 255))

                self.screen.blit(mass_label, (10, 10))
                self.screen.blit(velocity_label_x_and_y, (10, 30))
                self.screen.blit(type_label, (10, 50))
                self.screen.blit(position_label, (10, 70))

        #preview mode drawing
        if self.preview_mode and self.dummy_body is not None:
            radius = self.get_scaled_radius(self.dummy_body)
            pygame.draw.circle(self.screen, self.dummy_body.render()['color'], self.dummy_body.position, int(radius))
            pygame.draw.line(self.screen, (255, 255, 255), self.dummy_body.position, pygame.mouse.get_pos(), 2)


        if self.dummy_body is not None:
            preview_mass_label = self.font.render(f"Mass: {self.dummy_body.mass:.2e} kg", True, (255, 255, 255))
            preview_velocity_label_x_and_y = self.font.render(f"Velocity: {self.dummy_body.velocity[0]:.2e} m/s, {-(self.dummy_body.velocity[1]):.2e} m/s", True, (255, 255, 255))  #adding a -ve sign to the y velocity
            preview_type_label = self.font.render(f"Type: {self.dummy_body.status}", True, (255, 255, 255))
            position  = self.get_position_in_simulation(self.dummy_body.position)
            preview_position_label = self.font.render(f"Position: {position[0]:.2e} m, {position[1]:.2e} m", True, (255, 255, 255))
            
            self.screen.blit(preview_mass_label, (self.WIDTH - 300, 10))
            self.screen.blit(preview_velocity_label_x_and_y, (self.WIDTH - 300, 30))
            self.screen.blit(preview_type_label, (self.WIDTH - 300, 50))
            self.screen.blit(preview_position_label, (self.WIDTH - 300, 70))

            if self.focus_index != -1:
                escape_velocity = self.calculate_escape_velocity(self.bodies[self.focus_index].mass,self.dummy_body.mass,np.linalg.norm(self.bodies[self.focus_index].position - position))
                orbital_velocity = self.calculate_orbital_velocity_x_y(self.bodies[self.focus_index].mass,self.dummy_body.mass,np.linalg.norm(self.bodies[self.focus_index].position - position))
                preview_escape_velocity_label = self.font.render(f"Escape Velocity: {escape_velocity:.2e} m/s", True, (255, 255, 255))
                preview_orbital_velocity_label = self.font.render(f"Orbital Velocity: {orbital_velocity:.2e} m/s", True, (255, 255, 255))
                self.screen.blit(preview_escape_velocity_label, (self.WIDTH - 300, 90))
                self.screen.blit(preview_orbital_velocity_label, (self.WIDTH - 300, 110))

        #add scale and zoom labels
        label_scale = self.font.render(f"Scale: {self.SCALE:.2e} m", True, (255, 255, 255))
        label_day = self.font.render(f"Time per frame: {(self.TIME/self.DAY):.2e} day", True, (255, 255, 255))
                   
        self.screen.blit(label_scale, (10, self.HEIGHT - 50))
        self.screen.blit(label_day, (self.WIDTH-190, self.HEIGHT - 50))

        #add a pause label at the center top of the screen when paused
        if not self.simulation_running:
            label = self.font.render("Paused", True, (255, 255, 255))
            
            #similarly add a preview mode label at the center top of the screen when in preview mode
            if self.preview_mode:
                label = self.font.render("Preview Mode", True, (255, 255, 255))
            self.screen.blit(label, (self.WIDTH//2 - 30, 10))
        else:
            label = self.font.render(f"Time: {self.getTime()}", True, (255, 255, 255))
            self.screen.blit(label, (10, self.HEIGHT - 30))

    def camera_update(self):
        if self.focus_index != -1:
            self.camera_position = self.bodies[self.focus_index].position[:2].copy()
    
    def reset_bodies(self):
        self.bodies = []
        self.focus_index = -1
        self.TIME_PASSED = 0
        self.TIME = 24 * 60 * 60  #1 day initially
        self.simulation_running = False

    def run(self):
        running = True
        mouse_down_time = 0
        click_position = None
        while running:
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN: 

                    #load solar system
                    if event.key == pygame.K_1:
                        self.load_solar_system()
                    elif event.key == pygame.K_2:
                        self.initialize_alpha_centauri_system()
                    elif event.key == pygame.K_3:
                        self.load_particle_cloud()
                    #Scale control
                    if event.key == pygame.K_EQUALS:  # Zoom in
                        self.SCALE = max(self.SCALE / 1.1, 1e3)  # Clamp to a minimum scale
                    elif event.key == pygame.K_MINUS:  # Zoom out
                        self.SCALE = min(self.SCALE * 1.1, 1e24) 
                    
                    #Time Control
                    elif event.key == pygame.K_UP:
                        self.TIME = min(24*60*60,self.TIME*1.1) #speed up the simulation
                    elif event.key == pygame.K_DOWN:
                        self.TIME /= 1.1 #slow down the simulation

                    #pause control
                    elif event.key == pygame.K_SPACE:  # Pause the simulation
                        self.simulation_running = not self.simulation_running

                    #camera control
                    elif event.key == pygame.K_w:  # Move camera up
                        if self.focus_index == -1:
                            self.camera_position[1] += 50 * self.SCALE
                    elif event.key == pygame.K_s:  # Move camera down
                        if self.focus_index == -1:
                            self.camera_position[1] -= 50 *self.SCALE
                    elif event.key == pygame.K_a:  # Move camera left
                        if self.focus_index == -1:
                            self.camera_position[0] -= 50 * self.SCALE
                    elif event.key == pygame.K_d:  # Move camera right
                        if self.focus_index == -1:
                            self.camera_position[0] += 50 * self.SCALE
                    
                    #focus control
                    elif event.key == pygame.K_RIGHT:  # Toggle focus between bodies
                        if self.bodies:  # Ensure there are bodies to focus on
                            self.focus_index = (self.focus_index + 1) % len(self.bodies)
                    elif event.key == pygame.K_LEFT:  # Toggle focus between bodies
                        if self.bodies:  # Ensure there are bodies to focus on
                            self.focus_index = (self.focus_index - 1) % len(self.bodies)
                    elif event.key == pygame.K_u:  # Unfocus the camera
                        self.focus_index = -1

                    elif event.key == pygame.K_x:  # Delete the focused body
                        if self.focus_index != -1:
                            del self.bodies[self.focus_index]
                            if self.bodies:
                                self.focus_index = (self.focus_index - 1) % len(self.bodies) # go back 1 body
                            else:
                                self.focus_index = -1

                    elif event.key == pygame.K_c:  # Change to star mode
                        self.current_mode = "star"
                    elif event.key == pygame.K_p:  # Change to planet mode
                        self.current_mode = "planet"
                    
                    
                    elif event.key == pygame.K_r:  # Reset the simulation
                        self.reset_bodies()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        #pause the sim when adding new objects
                        
                        self.preview_mode = True
                        mouse_down_time = time.time() #start tracking time
                        click_position = np.array(pygame.mouse.get_pos())

                        # Create a dummy body for preview
                        if self.current_mode == "star":
                            self.dummy_body = star(click_position, np.array([0, 0]), self.PROXIMA_MIN_MASS) #temp position,velocity and mass
                        else:
                            self.dummy_body = planet(click_position, np.array([0, 0]), self.min_mass_pluto) #temp position,velocity and mass


                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        if click_position is not None and self.preview_mode:
                            
                            mouse_down_time = 0
                            self.preview_mode = False
                            release_position = np.array(pygame.mouse.get_pos())
                            raw_velocity = (release_position - click_position)*1e3
                            raw_velocity[1] *= -1
                            if self.focus_index != -1:
                                raw_velocity = self.bodies[self.focus_index].velocity + raw_velocity
                            velocity = raw_velocity
                            mass = self.dummy_body.mass
                            position = self.get_position_in_simulation(click_position)
                            self.add_body(position, velocity, mass, self.current_mode)
                            self.dummy_body = None
                            click_position = None


                elif event.type == pygame.MOUSEMOTION:
                    if self.preview_mode and click_position is not None:
                        drag_position = np.array(pygame.mouse.get_pos())
                        self.dummy_body.velocity = (drag_position - click_position)*1e3



            if self.preview_mode and click_position is not None:
                hold_duration = time.time() - mouse_down_time
                if self.current_mode == "star":
                    preview_mass = self.PROXIMA_MIN_MASS + hold_duration * 2e29
                else:
                    max_hold_time = 20
                    mass_range = self.max_mass_jupiter - self.min_mass_pluto
                    progress = min(hold_duration / max_hold_time, 1)
                    scaled_increment = mass_range * (progress ** 4)
                    preview_mass = min(self.min_mass_pluto + scaled_increment, self.max_mass_jupiter)
                self.dummy_body.mass = preview_mass
                self.dummy_body.mass_to_radius()
                        
                            
                            


            # Update the positions based on the forces
            if self.simulation_running and not self.preview_mode:     
                self.update_positions()

            self.camera_update()

            # Fill the background (you can choose any color or add a space-themed background)
            self.screen.fill((0, 0, 0))

            # Draw all bodies (stars and planets)
            self.draw_bodies()

            # Update the display
            pygame.display.flip()

            # Control the frame rate
            self.clock.tick(self.FPS)

        pygame.quit()

