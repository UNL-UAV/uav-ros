#include "core/Application.hpp"
extern Application* createApplication(int argc, char** argv);
int main(int argc, char** argv){
	Application* app = createApplication(argc, argv);
	app->run();
	delete app;
}
