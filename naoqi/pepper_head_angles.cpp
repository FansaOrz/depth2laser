// Aldebaran includes.
#include <alproxies/almotionproxy.h>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>
#include <qi/os.hpp>
// Boost includes.
#include <boost/program_options.hpp>
#include <thread>
#include <math.h>
#include <iostream>

using namespace std;
using namespace AL;
namespace po = boost::program_options;




bool stop_thread = false;

void head_angles_monitor_thread(qi::AnyObject motion_service, float head_yaw, float head_pitch) {

  float threshold = 0.1;
  while (!stop_thread){

    //TODO:read head angles
    vector<float> current_head_yaw = motion_service.call<vector<float>>("getAngles", ALValue("HeadYaw"), true);
    vector<float> current_head_pitch = motion_service.call<vector<float>>("getAngles", ALValue("HeadPitch"), true);

    //TODO: if current head_yaw or head_pitch difference > threshold
    //         set head angles to the desired angles
    if(abs(current_head_yaw[0] - head_yaw) >= threshold || abs(current_head_pitch[0] - head_pitch) >= threshold ) {
        motion_service.call<void>("setAngles", ALValue("HeadYaw"), ALValue(head_yaw), 1.0f);
        motion_service.call<void>("setAngles", ALValue("HeadPitch"), ALValue(head_pitch), 1.0f);
    }


    usleep(200000); //200ms
  }
  
  std::cerr << "Thread exit successfully" << std::endl;
}


int main(int argc, char **argv)
{
  std::string pepper_ip = "";
  if (std::getenv("PEPPER_IP") != NULL)
    pepper_ip = std::getenv("PEPPER_IP");

  po::options_description description("Options");
  description.add_options()
    ("help", "Displays this help message")
    ("pip", po::value<std::string>()->default_value(pepper_ip), "Robot IP address. Set IP here or for convenience, define PEPPER_IP as environment variable. On robot or Local Naoqi: use '127.0.0.1'.")
    ("pport", po::value<int>()->default_value(9559), "Naoqi port number.")
    ("head_yaw", po::value<float>()->default_value(0.2), "Pepper's desired head yaw angle.")
    ("head_pitch", po::value<float>()->default_value(-0.2), "Pepper's desired head pitch angle.")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);
  // --help option
  if (vm.count("help")){
    std::cout << description << std::endl;
    return 0;
  }

  //Params
  const std::string pip = vm["pip"].as<std::string>();
  int pport = vm["pport"].as<int>();
  float head_yaw = vm["head_yaw"].as<float>();
  float head_pitch = vm["head_pitch"].as<float>();

    if (pip == ""){
    std::cerr << "PEPPER_IP not defined. Please, set robot ip through program options" << std::endl;
    exit(0);
  }

  cerr << endl;
  cerr << "==========================" << endl;
  cerr << "pepper_ip: " << pip <<endl;
  cerr << "pepper_port: " << pport <<endl;
  cerr << "head_yaw: "<< head_yaw << endl;
  cerr << "head_pitch: "<< head_pitch << endl;
  cerr << "==========================" << endl << endl;

  std::string tcp_url("tcp://"+pip+":"+std::to_string(pport));

  qi::ApplicationSession app(argc, argv, 0, tcp_url);
  try {
    app.startSession();
  }
  catch (qi::FutureUserException e) {
    std::cerr << "Connection refused." << std::endl;
    exit(1);
  }
  
  //Init services
  qi::SessionPtr session = app.session();
  qi::AnyObject motion_service = session->service("ALMotion");

  //TODO: set stiffness head 1.0
  motion_service.call<void>("setStiffnesses", ALValue("Head"), 1.0f);
  //start thread
  std::thread ham_thread = std::thread(&head_angles_monitor_thread, motion_service, head_yaw, head_pitch);

  app.run();
  stop_thread = true;
  ham_thread.join();
  std::cerr << "Pepper head angles monitor exit successfully" << std::endl;
}
