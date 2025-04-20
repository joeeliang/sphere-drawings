import pygame
import sys
import math
import random

# --- Constants ---
WIDTH, HEIGHT = 800, 600
GRAVITY = pygame.math.Vector2(0, 981)  # Pixels per second^2
BACKGROUND_COLOR = (20, 20, 40)
BOUNCE_LOSS = 0.9 # Energy retained after bouncing off walls
COLLISION_DAMPING = 0.95 # Energy retained after inter-ball collision
MIN_RADIUS = 5
MAX_RADIUS = 20
SIMULATION_SUBSTEPS = 8 # Increase for more accuracy, decrease for performance
MAX_OBJECTS = 500 # Limit the number of objects

# --- Ball Class ---
class Ball:
    """Represents a single ball in the physics simulation."""
    def __init__(self, position, radius, color=None):
        self.position = pygame.math.Vector2(position)
        # Verlet integration uses current and previous position to infer velocity
        self.old_position = pygame.math.Vector2(position)
        self.acceleration = pygame.math.Vector2(0, 0)
        self.radius = radius
        self.mass = math.pi * radius**2 # Mass proportional to area
        self.color = color if color else (random.randint(100, 255), random.randint(100, 255), random.randint(100, 255))

    def update(self, dt):
        """Updates the ball's position using Verlet integration."""
        # Calculate velocity based on position change
        velocity = self.position - self.old_position

        # Store current position before updating
        self.old_position = pygame.math.Vector2(self.position)

        # print(self.acceleration)

        # Perform Verlet integration: pos += vel + acc * dt^2
        self.position += velocity + self.acceleration * dt * dt

        # Reset acceleration for the next frame/substep
        self.acceleration = pygame.math.Vector2(0, 0)

    def accelerate(self, force):
        """Applies a force to the ball (F = ma -> a = F/m)."""
        # Ensure mass is not zero to avoid division by zero
        if self.mass > 0:
            self.acceleration += GRAVITY

    def apply_constraints(self):
        """Keeps the ball within the screen boundaries."""
        velocity = self.position - self.old_position

        # Left boundary
        if self.position.x - self.radius < 0:
            self.position.x = self.radius
            self.old_position.x = self.old_position.x
        # Right boundary
        elif self.position.x + self.radius > WIDTH:
            self.position.x = WIDTH - self.radius
            self.old_position.x = self.old_position.x
        # Top boundary
        if self.position.y - self.radius < 0:
            self.position.y = self.radius
            self.old_position.y = self.old_position.y
        # Bottom boundary
        elif self.position.y + self.radius > HEIGHT:
            self.position.y = HEIGHT - self.radius
            self.old_position.y = self.old_position.y

    def draw(self, screen):
        """Draws the ball on the Pygame screen."""
        # Ensure position coordinates are integers for drawing
        draw_pos = (int(self.position.x), int(self.position.y))
        pygame.draw.circle(screen, self.color, draw_pos, int(self.radius))

# --- Simulation Class ---
class Simulation:
    """Manages the overall physics simulation."""
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Verlet Physics Simulation")
        self.clock = pygame.time.Clock()
        self.balls = []
        self.font = pygame.font.Font(None, 30)
        self.spawn_timer = 0  # Add timer for automatic spawning
        self.spawn_interval = 0.05  # Spawn a new ball every 0.1 seconds


    def add_ball(self, ball):
        """Adds a ball to the simulation, respecting the max object limit."""
        if len(self.balls) < MAX_OBJECTS:
            self.balls.append(ball)
        elif self.balls: # If limit reached, remove the oldest ball
             self.balls.pop(0)
             self.balls.append(ball)

    def apply_gravity(self):
        """Applies gravity to all balls."""
        for ball in self.balls:
            ball.accelerate(GRAVITY)

    def update_positions(self, dt):
        """Updates the position of each ball."""
        for ball in self.balls:
            ball.update(dt)

    def apply_constraints(self):
        """Applies screen boundary constraints to all balls."""
        for ball in self.balls:
            ball.apply_constraints()

    def solve_collisions(self):
        """Detects and resolves collisions between balls."""
        num_balls = len(self.balls)
        for i in range(num_balls):
            ball_1 = self.balls[i]
            for j in range(i + 1, num_balls):
                ball_2 = self.balls[j]

                # Vector from ball_1 center to ball_2 center
                collision_axis = ball_1.position - ball_2.position
                dist_sq = collision_axis.length_squared()
                min_dist = ball_1.radius + ball_2.radius

                # Check if balls are overlapping (using squared distance for efficiency)
                if dist_sq < min_dist * min_dist and dist_sq > 0: # Ensure dist_sq is not zero
                    dist = math.sqrt(dist_sq)
                    # Normalized collision axis
                    normal = collision_axis / dist
                    # Amount of overlap
                    overlap = (min_dist - dist) * 0.5 # Divide by 2 as both balls move

                    # Separate the balls based on mass (lighter moves more)
                    # Calculate total mass for mass ratio calculation
                    total_mass = ball_1.mass + ball_2.mass
                    if total_mass == 0: # Avoid division by zero if both masses are somehow zero
                         mass_ratio_1 = 0.5
                         mass_ratio_2 = 0.5
                    else:
                        mass_ratio_1 = ball_2.mass / total_mass if ball_1.mass > 0 else 1.0
                        mass_ratio_2 = ball_1.mass / total_mass if ball_2.mass > 0 else 1.0


                    # Move balls apart along the normal vector
                    separation_vector = normal * overlap
                    ball_1.position += separation_vector * mass_ratio_1 * COLLISION_DAMPING
                    ball_2.position -= separation_vector * mass_ratio_2 * COLLISION_DAMPING

                    # # --- Adjust old_position to conserve momentum visually ---
                    # # This part is crucial for Verlet integration to handle collisions correctly
                    # # Without adjusting old_position, balls might gain energy or stick together unnaturally
                    # vel1 = ball_1.position - ball_1.old_position
                    # vel2 = ball_2.position - ball_2.old_position

                    # # Reflect velocities along the collision normal (simplified elastic collision)
                    # relative_velocity = vel1 - vel2
                    # impulse_magnitude = relative_velocity.dot(normal)

                    # if total_mass > 0:
                    #     impulse = (-(1 + COLLISION_DAMPING) * impulse_magnitude / total_mass) * normal
                    #     # Apply impulse scaled by mass ratio
                    #     vel1 += impulse * ball_2.mass
                    #     vel2 -= impulse * ball_1.mass

                    # # Update old_position based on the new velocity after collision resolution
                    # ball_1.old_position = ball_1.position - vel1
                    # ball_2.old_position = ball_2.position - vel2


    def update(self, dt):
        """Performs a full simulation step, including sub-steps."""
        sub_dt = dt / SIMULATION_SUBSTEPS
        for _ in range(SIMULATION_SUBSTEPS):
            self.apply_gravity()
            self.solve_collisions()
            self.apply_constraints() # Apply constraints after collision solving
            self.update_positions(sub_dt)


    def draw(self):
        """Draws all elements of the simulation."""
        self.screen.fill(BACKGROUND_COLOR)
        for ball in self.balls:
            ball.draw(self.screen)

        # Display object count
        count_text = self.font.render(f"Objects: {len(self.balls)}/{MAX_OBJECTS}", True, (200, 200, 200))
        self.screen.blit(count_text, (10, 10))

        pygame.display.flip()

    def run(self):
            """Main simulation loop."""
            running = True
            while running:
                dt = self.clock.tick(60) / 1000.0  # Delta time in seconds

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # Automatic ball spawning
                self.spawn_timer += dt
                if self.spawn_timer >= self.spawn_interval:
                    self.spawn_timer = 0  # Reset timer
                    
                    # Create a ball at the center of the screen
                    center_pos = (WIDTH/2, HEIGHT/2)
                    radius = random.uniform(MIN_RADIUS, MAX_RADIUS)
                    new_ball = Ball(center_pos, radius)

                    # Calculate random angle for uniform circular distribution
                    angle = random.uniform(0, 2 * math.pi)
                    blast_speed = 200  # pixels/sec - adjust this value to change speed
                    
                    # Calculate velocity vector from angle
                    velocity = pygame.math.Vector2(
                        math.cos(angle) * blast_speed,
                        math.sin(angle) * blast_speed
                    )
                    
                    # Set old_position based on desired initial velocity for Verlet
                    new_ball.old_position = new_ball.position - velocity * dt
                    self.add_ball(new_ball)

                self.update(dt)
                self.draw()

            pygame.quit()
            sys.exit()

# --- Start Simulation ---
if __name__ == "__main__":
    simulation = Simulation()
    simulation.run()
