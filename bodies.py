import numpy as np
class body:
    def __init__(self, position, velocity, mass=None):
        self.position = np.array(position, dtype=np.float64)  # Ensure position is float64
        self.velocity = np.array(velocity, dtype=np.float64)  # Ensure velocity is float64
        self.mass = mass
        

    def render(self):
        color = self.mass_to_color()

        return {"position": self.position, "radius": self.radius, "color": color}
    

class planet(body):
    def __init__(self, position, velocity, mass):
        super().__init__(position, velocity, mass)
        self.status = 'planet'
        self.EARTH_MASS = 5.972e24  # kg
        self.EARTH_RADIUS = 6371e3  # km -> m
        self.JUPITER_MASS = 1.898e27  # kg
        self.JUPITER_RADIUS = 69911e3  # km -> m
        self.MASS_COLOR_SCALE = 1e25
        self.mass_to_radius()
        


    def mass_to_radius(self):
        if self.mass <= 2 * self.EARTH_MASS:  # Rocky planets
            self.radius = self.EARTH_RADIUS * (self.mass / self.EARTH_MASS) ** 0.28
        elif self.mass <= 0.1 * self.JUPITER_MASS:  # Sub-Neptunes
            self.radius = self.EARTH_RADIUS * (self.mass / self.EARTH_MASS) ** 0.59
        elif self.mass <= self.JUPITER_MASS:  # Gas giants
            # Scaling for gas giants with a slight positive power
            self.radius = self.JUPITER_RADIUS * (self.mass / self.JUPITER_MASS) ** 0.1
        else:  # Super-Jovian planets
            self.radius = self.JUPITER_RADIUS * (self.mass / self.JUPITER_MASS) ** -0.04

    def mass_to_color(self):
        if self.mass < 0.1 * self.MASS_COLOR_SCALE:  # Small rocky planets like Mercury
            return (105, 105, 105)  # Dark Gray
        elif self.mass < 0.5 * self.MASS_COLOR_SCALE:  # Larger rocky planets like Earth
            return (0, 100, 255)  # Earth-like blue
        elif self.mass < 1 * self.MASS_COLOR_SCALE:  # Ice giants like Neptune
            return (70, 130, 180)  # Steel Blue
        elif self.mass < 5 * self.MASS_COLOR_SCALE:  # Gas giants like Jupiter
            return (255, 165, 0)  # Orange
        elif self.mass < 10 * self.MASS_COLOR_SCALE:  # Super-Jovian or Brown Dwarf
            return (128, 0, 128)  # Purple (arbitrary for exotic planets)
        else:  # Extreme cases, e.g., failed stars
            return (255, 0, 0)  # Red for very massive objects


class star(body):
    def __init__(self, position, velocity, mass):
        super().__init__(position, velocity, mass)
        self.status = 'star'
        self.SUN_MASS = 1.989e30
        self.SUN_RADIUS = 6.9634e8
        self.MASS_COLOR_SCALE = 1e30 
        self.mass_to_radius()
        


    def mass_to_radius(self):
        if self.mass <= self.SUN_MASS:
            self.radius = self.SUN_RADIUS * (self.mass / self.SUN_MASS) ** 0.8
        if self.mass > self.SUN_MASS:
            self.radius = self.SUN_RADIUS*(self.mass / self.SUN_MASS) ** 0.57
    
    def mass_to_color(self):

        if self.mass < 1 * self.MASS_COLOR_SCALE:
            # Red → Orange
            return (255, int(165 * (self.mass / self.MASS_COLOR_SCALE)), 0)  
        elif self.mass < 2 * self.MASS_COLOR_SCALE:
            # Orange → Yellow
            return (255, 165 + int(90 * ((self.mass - self.MASS_COLOR_SCALE) / self.MASS_COLOR_SCALE)), 0)
        elif self.mass < 3 * self.MASS_COLOR_SCALE:
            # Yellow → White
            green = 255
            blue = int(255 * ((self.mass - 2 * self.MASS_COLOR_SCALE) / self.MASS_COLOR_SCALE))
            return (255, green, blue)
        elif self.mass < 4 * self.MASS_COLOR_SCALE:
            # White → Blue
            red = 255 - int(255 * ((self.mass - 3 * self.MASS_COLOR_SCALE) / self.MASS_COLOR_SCALE))
            return (red, 255, 255)
        else:
            # Max mass: Pure Blue
            return (0, 0, 255)
        

# sun = star(position=np.array([0, 0]), velocity=np.array([0, 0]), mass=1.989e30)
# earth = planet(position=np.array([1.5e11, 0]), velocity=np.array([0, 30000]), mass=5.972e24)
# jupiter = planet(position=np.array([7.785e11, 0]), velocity=np.array([0, 13000]), mass=1.898e27)  # Jupiter
# saturn  = planet(position=np.array([1.433e12, 0]), velocity=np.array([0, 9700]), mass=5.63e26)  # Saturn

# print(sun.render())
# print(earth.render())


# print(sun.radius/1e3)
# print(earth.radius/1e3)
# print(jupiter.radius/1e3)
# print(saturn.radius/1e3)

class particle(body):
    def __init__(self, position, velocity, mass):
        super().__init__(position, velocity, mass)
        self.status = 'particle'
        self.radius = self.mass_to_radius()
        
    def mass_to_radius(self):
        """
        Calculate the radius for a particle based on its mass.
        Particles are assumed to be very small compared to planets and stars.
        """
        return max(1e3, (self.mass / 1e20) ** (1/3) * 1e3)  # Scale radius proportionally to mass
    
    def mass_to_color(self):
        """
        Assign a uniform color to particles, representing their small size and lack of fusion activity.
        """
        return (169, 169, 169)  # Light Gray
    
    def render(self):
        """
        Render particle as a dictionary for visualization or output.
        """
        color = self.mass_to_color()
        return {"position": self.position, "radius": self.radius, "color": color}
