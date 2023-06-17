#include <memory>
#include <torch/script.h>
#include <utils.h>
#include <filesystem>

torch::jit::script::Module load_model() {
  
  std::string path = std::filesystem::current_path();
  size_t found = path.find("gscam");
  path.replace(found, 5, "model.pt");
  std::cout << "Absolute path to model is " << std::filesystem::absolute(path) << '\n';

  torch::jit::script::Module module;

  try {
    module = torch::jit::load(path);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    exit(0);
  }
  std::cout << "loaded model successfully\n";

  return module;
  /*
  std::vector<torch::jit::IValue> inputs;
inputs.push_back(torch::ones({1, 3, 224, 224}));

at::Tensor output = module.forward(inputs).toTensor();
std::cout << output << '\n';
"/home/hieuthenics/catkin_ws/src/mobilenet_model.pt"
*/
}

