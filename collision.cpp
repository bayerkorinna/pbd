#include <cmath>    
#include <iostream>
#include <array>
#include <tuple>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

int screen_dimx = 550;
int screen_dimy = 550;
int screen_leftx = -1;
int screen_rightx = 1;
int screen_topy = -1;
int screen_bottomy = 1;
int screen_world_width = screen_rightx-screen_leftx;
int screen_world_height = screen_bottomy-screen_topy;

float time_delta = 1.f / 64.f;
float particle_radii = 0.05f;
float last_time=0.f;
int dragged_particle = -1; // -1 means no particle is being dragged
bool is_dragging = false;
float particle_distance = particle_radii * 2.5f;

class Particle
{
private:
    float x;
    float y; 
    float vx = 0.f;
    float vy = 0.f;
    float px;
    float py;
    float r = particle_radii;
    float inv_mass = 1.f;
public:
    Particle()
        : x(0.0f), y(0.0f), px(0.0f), py(0.0f) {}

    Particle(float x_, float y_) 
        : x(x_), y(y_), px(x_), py(y_) {}

    ~Particle() = default;

    std::tuple<float,float> getPredPosition() {
        return std::tuple<float, float>(px, py);
    }

    float getRadius() const {
        return r;
    }

    float getX() const {
        return x;
    }

    float getY() const {
        return y;
    }

    float getVX() const { return vx; }
    float getVY() const { return vy; }
    float getPX() const { return px; }
    float getPY() const { return py; }
    float getInvMass() const { return inv_mass; }

    void setX(float x_) { x = x_; }
    void setY(float y_) { y = y_; }
    void setVX(float vx_) { vx = vx_; }
    void setVY(float vy_) { vy = vy_; }
    void setPX(float px_) { px = px_; }
    void setPY(float py_) { py = py_; }
    void setInvMass(float inv_mass_) { inv_mass = inv_mass_; } 
};

// initialize particles
const int num_particles = 10; // Number of particles in the simulation
std::array<Particle, num_particles> particles;
//int num_dragged_particles = 0;
//std::array<Particle, 1> dragged_particle_array;

// Remove the dragged_particle_array and use dragged_particle (float) to store the index of the dragged particle.
// Remove: std::array<Particle, 1> dragged_particle_array;

class PointConstraint
{
private:
    int id1;
    float x; 
    float y;
public:
    PointConstraint(int id1_, float x_, float y_)
        : id1(id1_), x(x_), y(y_) {}
        
    ~PointConstraint() = default;
};

void draw_circle_outline(float x, float y, float radius, int num_segments = 100)
{
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < num_segments; ++i)
    {
        float theta = 2.0f * M_PI * float(i) / float(num_segments);
        float dx = radius * cosf(theta);
        float dy = radius * sinf(theta);
        glVertex2f(x + dx, y + dy);
    }
    glEnd();
}

void draw_circle(float x, float y, float radius, int num_segments = 100)
{
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y); // Center of the circle
    for (int i = 0; i <= num_segments; ++i)
    {
        float theta = 2.0f * M_PI * float(i) / float(num_segments);
        float dx = radius * cosf(theta);
        float dy = radius * sinf(theta);
        glVertex2f(x + dx, y + dy);
    }
    glEnd();
}

void draw_particles()
{
    //std::cout << "Drawing particles..." << std::endl;
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    glColor3f(0.494, 0.616, 0.761);
    
    // Draw the particles
    for (const auto& particle : particles)
    {
        draw_circle(particle.getX(), particle.getY(), particle.getRadius());
        draw_circle_outline(particle.getX(), particle.getY(), particle.getRadius());
    }
    if (is_dragging && dragged_particle >= 0 && dragged_particle < num_particles) {
        // Draw the dragged particle with a different color or outline
        //std::cout << "Dragging particle index: " << dragged_particle << std::endl;
        const Particle& dp = particles[dragged_particle];
        glColor3f (1.0, 0.0, 0.0);
        draw_circle_outline(dp.getX(), dp.getY(), dp.getRadius());
    }
} 

float distance(float x1, float y1, float x2, float y2) {
    return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

std::tuple<float,float> point_constraint(Particle particle1, float x2, float y2)
{
    float correction_x1 = 0.0f;
	float correction_y1 = 0.0f;
	float px1 = std::get<0>(particle1.getPredPosition()); //predicted x position of particle 1
	float py1 = std::get<1>(particle1.getPredPosition());  //predicted y position of particle 1
	float coef1 = - 1.0;
	float coef2 = 0.0;
	float currdist = distance(px1,py1,x2,y2);
	if (currdist > 0.001) 
	{
        float dist_diff = currdist; // - constraint_distance
		float coef = dist_diff / currdist;
		correction_x1 = coef1 * coef * (px1-x2);
		correction_y1 = coef1 * coef * (py1-y2);
    }
	return std::tuple<float,float>(correction_x1,correction_y1);
}

std::array<float,4> collision_constraint(Particle particle1, Particle particle2)
{
    float correction_x1 = 0.0;
	float correction_y1 = 0.0;
	float correction_x2 = 0.0;
	float correction_y2 = 0.0;

	float desiredDistance = particle1.getRadius()*2;

	float particleDist = distance(particle1.getX(), particle1.getY(), particle2.getX(), particle2.getY());

	//if particle1 is particle2, don't do any collision checking
	if (particleDist == 0.0) {return std::array<float,4>{correction_x1,correction_y1, correction_x2,correction_y2};}

	//if our distance is less than radius*2, we have a collision
	if (particleDist < desiredDistance-0.001)
    {
		//helpers, simple calculations we need
		float xDiff = particle1.getX() - particle2.getX();
        float yDiff = particle1.getY() - particle2.getY();

		float absXDiff = abs(xDiff);
		float absYDiff = abs(yDiff);

		//normal vector tells us direction of correction
		float normalVecX = xDiff / particleDist;
		float normalVecY = yDiff / particleDist ;

		//Corrections weighted according to inverse masses
		float p1InvMassCalc =  -1*(particle1.getInvMass())/(particle1.getInvMass() + particle2.getInvMass());
		float p2InvMassCalc = (particle2.getInvMass())/(particle1.getInvMass() + particle2.getInvMass());

		//constraint is how far we must adjust
		float constraint = particleDist - desiredDistance - 0.001;

		//apply correction factors
		correction_x1 = p1InvMassCalc * constraint * normalVecX;
		correction_x2 = p2InvMassCalc * constraint * normalVecX;

		correction_y1 = p1InvMassCalc * constraint * normalVecY;
		correction_y2 = p2InvMassCalc * constraint * normalVecY;
    }

	return std::array<float,4>{correction_x1,correction_y1,
	        correction_x2,correction_y2};
}

void resolve_collision_constraints()
{
    for (auto& p1 : particles)
    {
        for (auto& p2 : particles)
        {
            if (&p1 != &p2) // avoid self-collision
            {
                auto corrections = collision_constraint(p1, p2);
                float correction_x1 = corrections[0];
                float correction_y1 = corrections[1];
                float correction_x2 = corrections[2];
                float correction_y2 = corrections[3];

                // Apply the corrections to the particles
                p1.setPX(p1.getPX() + correction_x1);
                p1.setPY(p1.getPY() + correction_y1);
                p2.setPX(p2.getPX() + correction_x2);
                p2.setPY(p2.getPY() + correction_y2);
            }
        }
    }
}

void pbd_main_loop()
{
    float gravity = 0.f; // Gravity acceleration
    for (auto& p : particles)
    {
        p.setVX(p.getVX() + 0.0);
        p.setVY(p.getVY() + gravity * time_delta);
        //damp velocity
        p.setVX(p.getVX() * 0.8f);
        p.setVY(p.getVY() * 0.8f);
        //get initial predicted position
        p.setPX(p.getX() + p.getVX() * time_delta);
        p.setPY(p.getY() + p.getVY() * time_delta);
    }
    resolve_collision_constraints();
    for (auto& p : particles)
    {
        p.setVX((p.getPX() - p.getX()) / time_delta);
        p.setVY((p.getPY() - p.getY()) / time_delta);
        //update position
        p.setX(p.getPX());
        p.setY(p.getPY());
    }
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    draw_particles();
    glFlush();
}

int particle_clicked(float x, float y)
{
    //std::cout << "Checking for particle click at: (" << x << ", " << y << ")" << std::endl;
    for (size_t i = 0; i < particles.size(); ++i)
    {
        const Particle& particle = particles[i];
        //std::cout << "Checking particle " << i << " at position: (" << particle.getX() << ", " << particle.getY() << ")" << std::endl;
        if (distance(particle.getX(), particle.getY(), x, y) < particle.getRadius())
        {
            return static_cast<int>(i); // Return the index of the clicked particle
        }
    }
    return -1;
}

std::tuple<float, float> translate_to_world_coords(float screen_x, float screen_y)
{
    // Map screen_x from [0, screen_dimx] to [screen_leftx, screen_rightx]
    float world_x = screen_leftx + (screen_x / static_cast<float>(screen_dimx)) * screen_world_width;
    // Map screen_y from [0, screen_dimy] to [screen_topy, screen_bottomy]
    float world_y = screen_topy + ((screen_dimy - screen_y) / static_cast<float>(screen_dimy)) * screen_world_height;
    return std::make_tuple(world_x, world_y);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    //std::cout << "Mouse button callback triggered: button=" << button << ", action=" << action << std::endl;
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        float world_x, world_y;
        std::tie(world_x, world_y) = translate_to_world_coords(static_cast<float>(xpos), static_cast<float>(ypos));
        //std::cout << "Mouse clicked at screen coordinates: (" << xpos << ", " << ypos << ")" << std::endl;
        //std::cout << "Mouse clicked at world coordinates: (" << world_x << ", " << world_y << ")" << std::endl;

        int clicked_index = particle_clicked(world_x, world_y);
        if (clicked_index != -1) // If a particle was clicked
        {
            dragged_particle = clicked_index; // Store the index of the dragged particle
            is_dragging = true;
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
    {
        is_dragging = false;
        dragged_particle = -1;
    }
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (is_dragging && dragged_particle >= 0 && dragged_particle < num_particles)
    {
        float world_x, world_y;
        std::tie(world_x, world_y) = translate_to_world_coords(static_cast<float>(xpos), static_cast<float>(ypos));
        
        // Update the dragged particle's position
        particles[dragged_particle].setX(world_x);
        particles[dragged_particle].setY(world_y);
    }
}

int main() {
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    //Create a windowed mode window and its OpenGL context
    GLFWwindow* window = glfwCreateWindow(screen_dimx, screen_dimy, "Collisions in PBD Demo", nullptr, nullptr);
    if (!window)    
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    //make the windows context current
    glfwMakeContextCurrent(window);

    //set callbacks
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);

    // initialise particles
    for (size_t i = 0; i < num_particles; ++i)
    {
        particles[i] = Particle(-10 * particle_radii + i * particle_radii * 2.2f, 0.0f);
    }

    // main simulation loop
    while (!glfwWindowShouldClose(window))
    {
        pbd_main_loop();
        display();

        glfwSwapBuffers(window);
        glfwPollEvents();

        // render particles or perform other operations
        // for (const auto& particle : particles)
        // {
        //     std::cout << "Particle Position: (" << particle.getX() << ", " << particle.getY() << ")\n";
        // }
    }
    glfwTerminate();
    return 0;
}
