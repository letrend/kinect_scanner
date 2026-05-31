#include "ncurses_3dscanner.hpp"
#include <cstring>
#include <cstdio>

#ifdef BUILD_GUI
#  include "gui_entry.hpp"
#else
static int runGui(int /*argc*/, char* /*argv*/[]) {
	std::fprintf(stderr, "GUI support not built. Re-configure with -DBUILD_GUI=ON.\n");
	return 2;
}
#endif

static int runCli() {
	VolumeIntegration scanner;
	if(scanner.intializeGridPosition()){
		scanner.scan();
		scanner.extractMesh();
		scanner.saveMesh();
		return 0;
	}
	std::cout << "could not initialize grid location" << std::endl;
	return 1;
}

int main(int argc, char *argv[]) {
	bool useGui = true;
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--cli") == 0) {
			useGui = false;
		} else if (std::strcmp(argv[i], "--gui") == 0) {
			useGui = true;
		} else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
			std::printf("Usage: %s [--gui|--cli] [--simulate|--actuated] [--sim-stl PATH]\n"
			            "  --gui   Launch the Qt GUI (default)\n"
			            "  --cli   Run the legacy ncurses/OpenCV CLI\n"
			            "  --simulate       Launch GUI with simulated Kinect/turntable\n"
			            "  --actuated       Launch GUI with actuator TCP pose source\n"
			            "  --sim-stl PATH   STL model for simulation\n"
			            "  --control-tcp [HOST:]PORT  UI control TCP endpoint\n"
			            "  --actuator-tcp [HOST:]PORT Actuator TCP endpoint\n",
			            argv[0]);
			return 0;
		}
	}
	return useGui ? runGui(argc, argv) : runCli();
}
