#include <iostream>
#include <filesystem>
namespace fs = std::filesystem;
 
int main()
{
    std::string path = std::filesystem::current_path();
    std::cout << "Current path is " << path << '\n';
    size_t found = path.find("gscam");
    path.replace(found, 5, "model.pt");
    std::cout << "Absolute path for " << path << " is " << fs::absolute(path) << '\n';
}