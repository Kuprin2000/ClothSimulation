// This is a personal academic project. Dear PVS-Studio, please check it.

// PVS-Studio Static Code Analyzer for C, C++, C#, and Java: https://pvs-studio.com

#include <iostream>
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include "implot/implot.h"
#include "simulation_controller.h"
#include "defines.h"
#include "bool_vector.h"

#ifdef PERFORMANCE_TEST
#include <chrono>
#endif

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window, SimulationController& controller);
void mouseCallback(GLFWwindow* window, double x_pos, double y_pos);

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;

bool FIRST_MOUSE = true;
float PREV_MOUSE_X = (float)WINDOW_WIDTH / 2.0f;
float PREV_MOUSE_Y = (float)WINDOW_HEIGHT / 2.0f;
float MOUSE_X = (float)WINDOW_WIDTH / 2.0f;
float MOUSE_Y = (float)WINDOW_HEIGHT / 2.0f;

int main()
{
	// init opengl
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef MSAA4
	glfwWindowHint(GLFW_SAMPLES, 4);
#endif

	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Cloth simulation", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetCursorPosCallback(window, mouseCallback);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

#ifdef V_SYNC
	glfwSwapInterval(1);
#else
	glfwSwapInterval(0);
#endif 

#ifdef MSAA4
	glEnable(GL_MULTISAMPLE);
#endif
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);

	// init imgui
	ImGui::CreateContext();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330");
	ImPlot::CreateContext();

	// init model and controller
	SimulationModel model;
	SimulationView view;
	SimulationController controller(&model, &view);


#ifdef PERFORMANCE_TEST
	int frame_number = 0;
	auto begin = std::chrono::high_resolution_clock::now();
#endif 

	omp_set_dynamic(0);
	omp_set_num_threads(THREADS_COUNT);

	// main loop
	bool need_close = false;

#pragma omp parallel
	{
		while (!need_close)
		{
#pragma omp master
			{
				need_close = glfwWindowShouldClose(window);
				processInput(window, controller);
			}

			controller.newFrame();

#pragma omp master
			{
				glfwSwapBuffers(window);
				glfwPollEvents();

#ifdef PERFORMANCE_TEST
				if (!frame_number)
				{
					begin = std::chrono::high_resolution_clock::now();
				}

				frame_number += (int)(controller.getState() == SimulationState::SIMULATION_RUN);

#endif 
			}

#ifdef PERFORMANCE_TEST

#pragma omp master
			{
				need_close |= (frame_number == FRAMES_COUNT);
			}
#pragma omp barrier
#endif 
		}
	}

#ifdef PERFORMANCE_TEST
	const auto end = std::chrono::high_resolution_clock::now();
	const int time = (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
	const ModelSettings& settings = model.getSettings();

	std::cout << "Settings: " << std::endl;
	std::cout << "Constraints threads count " << THREADS_COUNT << std::endl;
	std::cout << "Preferred partitions count " << settings.m_preferred_partitions_count << std::endl;
	std::cout << "Iterations count " << settings.m_iterations_count << std::endl;

	std::cout << std::endl;

	std::cout << "Test results: " << std::endl;
	std::cout << FRAMES_COUNT << " frames at " << time << " milliseconds" << std::endl;
	std::cout << "Average framerate " << controller.getAverageFramerate() << std::endl;
	std::cout << "Average evaluate forces time " << controller.getAverageEvaluateForcesTime() << std::endl;
	std::cout << "Total evaluate forces time " << controller.getTotalEvaluateForcesTime() << std::endl;
	std::cout << "Average r-tree creation time " << controller.getAverageRTreeCreationTime() << std::endl;
	std::cout << "Total r-tree creation time " << controller.getTotalRTreeCreationTime() << std::endl;
	std::cout << "Average find collision candidates time " << controller.getAverageFindCollisionCandidatesTime() << std::endl;
	std::cout << "Total find collision candidates time " << controller.getTotalFindCollisionCandidatesTime() << std::endl;
	std::cout << "Average check collision candidates time " << controller.getAverageCheckCollisionCandidatesTime() << std::endl;
	std::cout << "Total check collision candidates time " << controller.getTotalCheckCollisionCandidatesTime() << std::endl;
	std::cout << "Average collision constraints graph time " << controller.getAverageCollisionConstraintsGraphTime() << std::endl;
	std::cout << "Total collision constraints graph time " << controller.getTotalCollisionConstraintsGraphTime() << std::endl;
	std::cout << "Average user constraints graph time " << controller.getAverageUserConstraintsGraphTime() << std::endl;
	std::cout << "Total user constraints graph time " << controller.getTotalUserConstraintsGraphTime() << std::endl;
	std::cout << "Average evaluate constraints and friction time " << controller.getAverageEvaluateConstraintsTime() << std::endl;
	std::cout << "Total evaluate constraints and friction time " << controller.getTotalEvaluateConstraintsTime() << std::endl;
	std::cout << "Average update positions and speeds time " << controller.getAverageUpdatePositionsAndSpeedsTime() << std::endl;
	std::cout << "Total update positions and speeds time " << controller.getTotalUpdatePositionsAndSpeedsTime() << std::endl;
	std::cout << "Average update normals time " << controller.getAverageUpdateNormalsTime() << std::endl;
	std::cout << "Total update normals time " << controller.getTotalUpdateNormalsTime() << std::endl;
	std::cout << "Average collision constraints count " << controller.getAverageCollisionConstraitnsCount() << std::endl;
	std::cout << "Total collision constraints count " << controller.getTotalCollisionConstraitnsCount() << std::endl;
#endif 

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void processInput(GLFWwindow* window, SimulationController& controller)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, 1);
	}

	const bool fast_movement = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::FRONT, fast_movement);
	}

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::BACK, fast_movement);
	}

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::LEFT, fast_movement);
	}

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::RIGHT, fast_movement);
	}

	if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::UP, fast_movement);
	}

	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{
		controller.moveCamera(MovementDirection::DOWN, fast_movement);
	}

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT))
	{
		controller.rotateCamera(MOUSE_X - PREV_MOUSE_X, PREV_MOUSE_Y - MOUSE_Y);
	}

	PREV_MOUSE_X = MOUSE_X;
	PREV_MOUSE_Y = MOUSE_Y;
}

void mouseCallback(GLFWwindow* window, double x_pos, double y_pos)
{
	PREV_MOUSE_X = MOUSE_X;
	PREV_MOUSE_Y = MOUSE_Y;

	MOUSE_X = (float)x_pos;
	MOUSE_Y = (float)y_pos;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}
