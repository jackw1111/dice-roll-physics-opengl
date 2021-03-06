#include <iostream>

#include "glad.h"
#include <GLFW/glfw3.h>
#include "stb_image.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "shader.h"
#include "camera.h"
#include "model.h"
#include "physics.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
unsigned int loadTexture(const char *path);
void renderQuad();
glm::mat4 generateShadowMap();

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 10.0f, 15.0f), glm::vec3(0.0f, 1.0f, 0.0f), YAW, -25.0f);
float lastX = (float)SCR_WIDTH / 2.0;
float lastY = (float)SCR_HEIGHT / 2.0;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// meshes
unsigned int planeVAO;

glm::vec3 lightPos(-2.0f, 4.0f, -1.0f);

float rotation1_x = 0.0f;
float rotation1_y = 0.0f;
float rotation1_z = 0.0f;

unsigned int maxContacts = 256;
Contact contacts[256];
CollisionData cData;
ContactResolver resolver;
unsigned int boxes = 3;
CollisionBox* boxData[3];

class Plane {
public:
    Plane(glm::vec3 n, glm::mat4 m) {
        normal = n;
        modelMatrix = m;
    }
    glm::vec3 normal;
    glm::mat4 modelMatrix;
};

class Die {
public:
    glm::mat4 modelMatrix;

    Model *model;

    Die(glm::vec3 pos, glm::vec3 vel) {
        model = new Model("data/die.obj", false);
    }

    void draw(Shader &shader) {
        shader.setMat4("model", modelMatrix);
        model->Draw(shader);    
    }
};

Plane plane(glm::vec3(0,1,0), glm::mat4(1.0f));

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfwGetPrimaryMonitor()
    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    // build and compile shaders
    // -------------------------
    Shader basicShader("shaders/basic_shader.vs", "shaders/basic_shader.fs", nullptr);
    Shader shadowMapShader("shaders/shadow_map_shader.vs", "shaders/shadow_map_shader.fs", nullptr);
    Shader normalShader("shaders/normal_map_shader.vs", "shaders/normal_map_shader.fs", nullptr);

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float planeVertices[] = {
        // positions            // normals         // texcoords
         25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,   0.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,

         25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
        -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,
         25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,  25.0f, 25.0f
    };
    // plane VAO
    unsigned int planeVBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glBindVertexArray(0);

    // load textures
    // -------------
    unsigned int diffuseMap = loadTexture("data/brickwall.jpg");
    unsigned int normalMap  = loadTexture("data/brickwall_normal.jpg");


    // configure depth map FBO
    // -----------------------
    const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
    unsigned int depthMapFBO;
    glGenFramebuffers(1, &depthMapFBO);
    // create depth texture
    unsigned int depthMap;
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    // attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // shader configuration
    // --------------------
    basicShader.use();
    basicShader.setInt("diffuseTexture", 0);
    basicShader.setInt("shadowMap", 1);
    basicShader.setInt("normalMap", 2);

    normalShader.use();
    normalShader.setInt("diffuseMap", 0);
    normalShader.setInt("normalMap", 1);
    normalShader.setInt("shadowMap", 2);

    plane.modelMatrix = glm::scale(plane.modelMatrix, glm::vec3(10.0f));
    plane.modelMatrix = rotate(plane.modelMatrix, glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));

    Die die1(glm::vec3(2.0f, 5.0f, 0.0f), glm::vec3(0.6f, 0.0f, 0.0f));
    Die die2(glm::vec3(-2.0f, 5.0f, 0.0f), glm::vec3(0.8f, 0.0f, 0.0f));

    // PHYSICS
    resolver = ContactResolver(maxContacts*8);
    cData.contactArray = contacts;
    // reset box states
    CollisionBox box1 = CollisionBox(0.0, 0.0, 0.0, glm::vec3(0,0,0), 0.0, 0.0, 0.0, 0.0, 100000.0, 1.0, 100000.0);
    CollisionBox box2 = CollisionBox(0.0, 3.0, 0.0, glm::vec3(0,0,0), -10.0, 4.0, 1.0, 3.0, 1.0, 1.0, 1.0);
    CollisionBox box3 = CollisionBox(0.0, 6.5, 0.0, glm::vec3(0,0,0), -10.0, 1.0, 2.0, 4.0, 1.0, 1.0, 1.0);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // physics
        // ------
        if (deltaTime < 1.0) {

            CollisionBox::boxData[0]->setState(0.0, 0.0, 0.0, glm::vec3(rotation1_y, 0, rotation1_x), 0.0, 0.0, 0.0, 0.0, 10.0, 0.1, 10.0);

            // Update the boxes
            for (unsigned int i = 0; i < CollisionBox::boxCount; i++)
            {
                CollisionBox *box = CollisionBox::boxData[i];

                // Run the physics
                box->body->integrate(deltaTime);
                box->calculateInternals();
            }
            
            // Perform the contact generation

            // Set up the collision data structure
            cData.reset(maxContacts);
            cData.friction = (double)0.9;
            cData.restitution = (double)0.1;
            cData.tolerance = (double)0.1;
            //CollisionBox::allBoxes.size()
            for (unsigned int i = 0; i < CollisionBox::boxCount; i++)
            {
                for (unsigned int j = 0; j < CollisionBox::boxCount; j++)
                {
                    CollisionBox *box1 = CollisionBox::boxData[i];
                    CollisionBox *box2 = CollisionBox::boxData[j];

                    if (box1 != box2) {
                        CollisionDetector::boxAndBox(*box1, *box2, &cData);
                    }
                }
            }

            // Resolve detected contacts
            resolver.resolveContacts(
                cData.contactArray,
                cData.contactCount,
                deltaTime
                );


            CollisionBox *box = CollisionBox::boxData[1];
            box->setModelMatrix(glm::scale((glm::mat4)box->body->transformMatrix, glm::vec3(box->halfSize.x*2, box->halfSize.y*2, box->halfSize.z*2)));
            die1.modelMatrix = glm::scale(box->getModelMatrix(), glm::vec3(0.5f));


            box = CollisionBox::boxData[2];
            box->setModelMatrix(glm::scale((glm::mat4)box->body->transformMatrix, glm::vec3(box->halfSize.x*2, box->halfSize.y*2, box->halfSize.z*2)));
            die2.modelMatrix = glm::scale(box->getModelMatrix(), glm::vec3(0.5f));
        }
        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 lightSpaceMatrix = generateShadowMap();


        // render scene from light's point of view
        shadowMapShader.use();
        shadowMapShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        shadowMapShader.setMat4("model", plane.modelMatrix);

        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
            glClear(GL_DEPTH_BUFFER_BIT);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, diffuseMap);
            renderQuad();
            die1.draw(shadowMapShader);
            die2.draw(shadowMapShader);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);


        // 2. render scene as normal using the generated depth/shadow map  
        // --------------------------------------------------------------
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

        normalShader.use();
        normalShader.setMat4("projection", projection);
        normalShader.setMat4("view", view);
        // set light uniforms
        normalShader.setVec3("viewPos", camera.Position);
        normalShader.setVec3("lightPos", lightPos);
        normalShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

        glm::mat4 model = glm::mat4(1.0f);

        //model = glm::rotate(model, glm::radians((float)glfwGetTime() * -10.0f), glm::normalize(glm::vec3(1.0, 0.0, 1.0))); // rotate the quad to show normal mapping from multiple directions
        normalShader.setMat4("model", plane.modelMatrix);
        normalShader.setVec3("viewPos", camera.Position);
        normalShader.setVec3("lightPos", lightPos);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, diffuseMap);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, normalMap);
        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        renderQuad();

        basicShader.use();
        projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        view = camera.GetViewMatrix();
        basicShader.setMat4("projection", projection);
        basicShader.setMat4("view", view);
        // set light uniforms
        basicShader.setVec3("viewPos", camera.Position);
        basicShader.setVec3("lightPos", lightPos);
        basicShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, diffuseMap);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, depthMap);


        die1.draw(basicShader);
        die2.draw(basicShader);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &planeVAO);
    glDeleteBuffers(1, &planeVBO);

    glfwTerminate();
    return 0;
}

// renders a 1x1 quad in NDC with manually calculated tangent vectors
// ------------------------------------------------------------------
unsigned int quadVAO = 0;
unsigned int quadVBO;
void renderQuad()
{
    if (quadVAO == 0)
    {
        // positions
        glm::vec3 pos1(-1.0f,  1.0f, 0.0f);
        glm::vec3 pos2(-1.0f, -1.0f, 0.0f);
        glm::vec3 pos3( 1.0f, -1.0f, 0.0f);
        glm::vec3 pos4( 1.0f,  1.0f, 0.0f);
        // texture coordinates
        glm::vec2 uv1(0.0f, 1.0f);
        glm::vec2 uv2(0.0f, 0.0f);
        glm::vec2 uv3(1.0f, 0.0f);  
        glm::vec2 uv4(1.0f, 1.0f);
        // normal vector
        glm::vec3 nm(0.0f, 0.0f, 1.0f);

        // calculate tangent/bitangent vectors of both triangles
        glm::vec3 tangent1, bitangent1;
        glm::vec3 tangent2, bitangent2;
        // triangle 1
        // ----------
        glm::vec3 edge1 = pos2 - pos1;
        glm::vec3 edge2 = pos3 - pos1;
        glm::vec2 deltaUV1 = uv2 - uv1;
        glm::vec2 deltaUV2 = uv3 - uv1;

        float f = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);

        tangent1.x = f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x);
        tangent1.y = f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y);
        tangent1.z = f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z);

        bitangent1.x = f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x);
        bitangent1.y = f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y);
        bitangent1.z = f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z);

        // triangle 2
        // ----------
        edge1 = pos3 - pos1;
        edge2 = pos4 - pos1;
        deltaUV1 = uv3 - uv1;
        deltaUV2 = uv4 - uv1;

        f = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);

        tangent2.x = f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x);
        tangent2.y = f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y);
        tangent2.z = f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z);


        bitangent2.x = f * (-deltaUV2.x * edge1.x + deltaUV1.x * edge2.x);
        bitangent2.y = f * (-deltaUV2.x * edge1.y + deltaUV1.x * edge2.y);
        bitangent2.z = f * (-deltaUV2.x * edge1.z + deltaUV1.x * edge2.z);


        float quadVertices[] = {
            // positions            // normal         // texcoords  // tangent                          // bitangent
            pos1.x, pos1.y, pos1.z, nm.x, nm.y, nm.z, uv1.x, uv1.y, tangent1.x, tangent1.y, tangent1.z, bitangent1.x, bitangent1.y, bitangent1.z,
            pos2.x, pos2.y, pos2.z, nm.x, nm.y, nm.z, uv2.x, uv2.y, tangent1.x, tangent1.y, tangent1.z, bitangent1.x, bitangent1.y, bitangent1.z,
            pos3.x, pos3.y, pos3.z, nm.x, nm.y, nm.z, uv3.x, uv3.y, tangent1.x, tangent1.y, tangent1.z, bitangent1.x, bitangent1.y, bitangent1.z,

            pos1.x, pos1.y, pos1.z, nm.x, nm.y, nm.z, uv1.x, uv1.y, tangent2.x, tangent2.y, tangent2.z, bitangent2.x, bitangent2.y, bitangent2.z,
            pos3.x, pos3.y, pos3.z, nm.x, nm.y, nm.z, uv3.x, uv3.y, tangent2.x, tangent2.y, tangent2.z, bitangent2.x, bitangent2.y, bitangent2.z,
            pos4.x, pos4.y, pos4.z, nm.x, nm.y, nm.z, uv4.x, uv4.y, tangent2.x, tangent2.y, tangent2.z, bitangent2.x, bitangent2.y, bitangent2.z
        };
        // configure plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);

    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
        CollisionBox::boxData[0]->setState(0.0, 0.0, 0.0, glm::vec3(0,0,0), 0.0, 0.0, 0.0, 0.0, 20.0, 0.1, 20.0);
        CollisionBox::boxData[1]->setState(0.0, 3.0, 0.0, glm::vec3(0,0,0), -10.0, 4.0, 1.0, 3.0, 1.0, 1.0, 1.0);
        CollisionBox::boxData[2]->setState(0.0, 6.5, 0.0, glm::vec3(0,0,0), -10.0, 1.0, 2.0, 4.0, 1.0, 1.0, 1.0);
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;
    plane.modelMatrix = glm::rotate(plane.modelMatrix, glm::radians(-1.0f*yoffset*0.01f), glm::vec3(1.0f, 0.0f, 0.0f));

    plane.modelMatrix = glm::rotate(plane.modelMatrix, glm::radians(xoffset*0.01f), glm::vec3(0.0f, 1.0f, 0.0f));


    glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(plane.modelMatrix)));
    glm::vec3 normal = glm::vec3(0.0f,0.0f,1.0f);
    plane.normal = glm::normalize(normalMatrix * normal);
    rotation1_x -= glm::radians(xoffset*0.01f);
    rotation1_y += glm::radians(-1.0f*yoffset*0.01f);

    //camera.ProcessMouseMovement(xoffset, yoffset, true);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const * path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat 
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}


glm::mat4 generateShadowMap() {
    float nearDist = 0.01f, farDist = 30.0f;
    float fov = glm::radians(camera.Zoom);

    float ar = (float)SCR_WIDTH / (float)SCR_HEIGHT;
    float Hnear = 2* tan(fov/2) * nearDist;
    float Wnear = Hnear * ar;
    float Hfar = 2* tan(fov/2) * farDist;
    float Wfar = Hfar * ar; 
    glm::vec3 centerFar = camera.Position + camera.Front * farDist;

    glm::vec3 topLeftFar = centerFar + (camera.Up * Hfar/2.0f) - (camera.Right * Wfar/2.0f);
    glm::vec3 topRightFar = centerFar + (camera.Up * Hfar/2.0f) + (camera.Right * Wfar/2.0f);
    glm::vec3 bottomLeftFar = centerFar - (camera.Up  * Hfar/2.0f) - (camera.Right * Wfar/2.0f);
    glm::vec3 bottomRightFar = centerFar - (camera.Up * Hfar/2.0f) + (camera.Right * Wfar/2.0f);

    glm::vec3 centerNear = camera.Position + camera.Front * nearDist;

    glm::vec3 topLeftNear = centerNear + (camera.Up * Hnear/2.0f) - (camera.Right * Wnear/2.0f);
    glm::vec3 topRightNear = centerNear + (camera.Up * Hnear/2.0f) + (camera.Right * Wnear/2.0f);
    glm::vec3 bottomLeftNear = centerNear - (camera.Up * Hnear/2.0f) - (camera.Right * Wnear/2.0f);
    glm::vec3 bottomRightNear = centerNear - (camera.Up * Hnear/2.0f) + (camera.Right * Wnear/2.0f);

    glm::vec3 frustumCenter = (centerFar- centerNear)*0.5f;

    glm::mat4 lightView = glm::lookAt(glm::normalize(lightPos), glm::vec3(0,0,0), glm::vec3(0,0,1));

    std::array<glm::vec3, 8> frustumToLightView
    {
        lightView * glm::vec4(bottomRightNear, 1.0f),
        lightView * glm::vec4(topRightNear, 1.0f),
        lightView * glm::vec4(bottomLeftNear, 1.0f),
        lightView * glm::vec4(topLeftNear, 1.0f),
        lightView * glm::vec4(bottomRightFar, 1.0f),
        lightView * glm::vec4(topRightFar, 1.0f),
        lightView * glm::vec4(bottomLeftFar, 1.0f),
        lightView * glm::vec4(topLeftFar, 1.0f)
    };

    // find max and min points to define a ortho matrix around
    glm::vec3 min{ INFINITY, INFINITY, INFINITY };
    glm::vec3 max{ -INFINITY, -INFINITY, -INFINITY };
    for (unsigned int i = 0; i < frustumToLightView.size(); i++)
    {
        if (frustumToLightView[i].x < min.x)
            min.x = frustumToLightView[i].x;
        if (frustumToLightView[i].y < min.y)
            min.y = frustumToLightView[i].y;
        if (frustumToLightView[i].z < min.z)
            min.z = frustumToLightView[i].z;

        if (frustumToLightView[i].x > max.x)
            max.x = frustumToLightView[i].x;
        if (frustumToLightView[i].y > max.y)
            max.y = frustumToLightView[i].y;
        if (frustumToLightView[i].z > max.z)
            max.z = frustumToLightView[i].z;
    }

    float l = min.x;
    float r = max.x;
    float b = min.y;
    float t = max.y;
    // because max.z is positive and in NDC the positive z axis is 
    // towards us so need to set it as the near plane flipped same for min.z.
    float n = -max.z;
    float f = -min.z;

    // finally, set our ortho projection
    // and create the light space view-projection matrix
    glm::mat4 lightProjection = glm::ortho(l,r,b,t,n,f);
    glm::mat4 lightSpaceMatrix = lightProjection * lightView;
    return lightSpaceMatrix;
}
