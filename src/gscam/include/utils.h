#ifndef __tst_H
#define __tst_H

#include <memory>
#include <torch/script.h>
#include <utils.h>
#include <filesystem>


torch::jit::script::Module load_model();

#endif
