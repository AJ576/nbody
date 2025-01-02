import time
import pygame
import random
import math
import numpy as np
from bodies import body, planet, star

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

        

        # self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        # self.slider_rect = pygame.Rect(self.WIDTH // 2 - self.SLIDER_WIDTH // 2, self.HEIGHT - 50, self.SLIDER_WIDTH, self.SLIDER_HEIGHT)
        # self.slider_thumb_rect = pygame.Rect(self.WIDTH // 2 - self.SLIDER_WIDTH // 2 + self.TIMESTEP // 1000, self.HEIGHT - 55, 10, 20)

        # self.pygame.display.set_caption("Interactive Three-Body Simulation")
        # self.clock = pygame.time.Clock()
        # self.font = pygame.font.SysFont("Arial", 16)

    def add_body(self, position, velocity, mass=None,status=None):
        if status == "star":
            mass if mass else self.PROXIMA_MIN_MASS
            new_body = star(position, velocity, mass)
        else:
            mass = min(self.max_mass_jupiter, mass)
            new_body = planet(position, velocity, mass)
        self.bodies.append(new_body)
        # self.focus_index = len(self.bodies) - 1
    
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
        forces = self.compute_forces()
        for i, body in enumerate(self.bodies):
            accelration = forces[i] / body.mass
            body.position += body.velocity * self.TIME + 0.5 * accelration * self.TIME**2
            body.velocity += accelration * self.TIME

    def get_scaled_radius(self,body):
        radius = body.radius / self.SCALE
        return max(radius, 1)

    def get_position_in_simulation(self, position):
        screen_position = ((np.array(position) - np.array([self.WIDTH / 2, self.HEIGHT / 2])) * self.SCALE) + self.camera_position
        screen_position[1] *= -1  # Flip the y-coordinate
        return screen_position

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
            pygame.draw.circle(self.screen, color, (x, y), int(radius))


            #preview mode drawing
            if self.preview_mode and self.dummy_body is not None:
                radius = self.get_scaled_radius(self.dummy_body)
                pygame.draw.circle(self.screen, self.dummy_body.render()['color'], self.dummy_body.position, int(radius))

                pygame.draw.line(self.screen, (255, 255, 255), self.dummy_body.position, pygame.mouse.get_pos(), 2)

            # Optionally, add labels for bodies
            
            if body.status == 'star':
                label = self.font.render(f"Star: {body.mass:.2e} kg", True, (255, 255, 255))
                self.screen.blit(label, (x + 10, y + 10))
            else:
                if self.SCALE < 5e10: # only show labels for planets for small scales
                    label = self.font.render(f"Planet: {body.mass:.2e} kg", True, (255, 255, 255))
                    self.screen.blit(label, (x + 10, y + 10))

            #add scale and zoom labels
            label_scale = self.font.render(f"Scale: {self.SCALE:.2e} m", True, (255, 255, 255))
            label_day = self.font.render(f"Time per frame: {(self.TIME/self.DAY):.2e} day", True, (255, 255, 255))
            
            
                
            self.screen.blit(label_scale, (10, self.HEIGHT - 50))
            self.screen.blit(label_day, (self.WIDTH-190, self.HEIGHT - 50))

    def camera_update(self):
        if self.focus_index != -1:
            self.camera_position = self.bodies[self.focus_index].position[:2].copy()

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
                            self.camera_position[1] -= 50 * self.SCALE
                    elif event.key == pygame.K_s:  # Move camera down
                        if self.focus_index == -1:
                            self.camera_position[1] += 50 *self.SCALE
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

                    elif event.key == pygame.K_c:  # Clear all bodies
                        self.current_mode = "star"
                    elif event.key == pygame.K_p:  # Change to planet mode
                        self.current_mode = "planet"
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
                    preview_mass = self.PROXIMA_MIN_MASS + hold_duration * 3e29
                else:
                    max_hold_time = 15
                    mass_range = self.max_mass_jupiter - self.min_mass_pluto
                    progress = min(hold_duration / max_hold_time, 1)
                    scaled_increment = mass_range * (progress ** 4)
                    preview_mass = min(self.min_mass_pluto + scaled_increment, self.max_mass_jupiter)
                self.dummy_body.mass = preview_mass
                        
                            
                            


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

if __name__ == "__main__":
    sim = simulation()

    # Sun (already added)
    sim.add_body(position=np.array([0, 0]), velocity=np.array([0, 0]), mass=1.989e30,status = "star")  # Sun

    # Mercury
    sim.add_body(position=np.array([5.79e10, 0]), velocity=np.array([0, 47400]), mass=3.3011e23,status = "planet")  # Mercury

    # Venus
    sim.add_body(position=np.array([1.082e11, 0]), velocity=np.array([0, 35020]), mass=4.867e24,status = "planet")  # Venus

    # Earth (already added)
    sim.add_body(position=np.array([1.5e11, 0]), velocity=np.array([0, 30000]), mass=5.972e24,status = "planet")  # Earth

    # Mars
    sim.add_body(position=np.array([2.279e11, 0]), velocity=np.array([0, 24000]), mass=6.417e23,status = "planet")  # Mars

    # Jupiter
    sim.add_body(position=np.array([7.785e11, 0]), velocity=np.array([0, 13000]), mass=1.898e27,status = "planet")  # Jupiter

    # Saturn
    sim.add_body(position=np.array([1.433e12, 0]), velocity=np.array([0, 9700]), mass=5.683e26,status = "planet")  # Saturn

    # Uranus
    sim.add_body(position=np.array([2.87e12, 0]), velocity=np.array([0, 6800]), mass=8.681e25,status = "planet")  # Uranus

    # Neptune
    sim.add_body(position=np.array([4.495e12, 0]), velocity=np.array([0, 5400]), mass=1.024e26,status = "planet")  # Neptune

    # Pluto (dwarf planet)
    sim.add_body(position=np.array([5.906e12, 0]), velocity=np.array([0, 4700]), mass=1.303e22,status = "planet")  # Pluto
    # Run the simulation
    sim.run()