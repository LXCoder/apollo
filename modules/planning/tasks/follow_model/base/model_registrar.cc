#include "model_registrar.h"

namespace apollo {
namespace planning {

ModelRegistrar::ModelRegistrar()
    : model_factory_(
          std::make_shared<Factory<FollowModelBase, std::string>>()) {}

std::shared_ptr<FollowModelBase> ModelRegistrar::CreateProduct(
    const std::string& model_name) {
  return std::move(model_factory_->CreateProduct(model_name));
}

}  // namespace planning
}  // namespace apollo