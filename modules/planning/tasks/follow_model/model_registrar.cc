#include "model_registrar.h"

#include "follow_model_idm.h"

#include "bazel-out/k8-dbg/bin/modules/planning/tasks/follow_model/proto/follow_model_config.pb.h"

namespace apollo {
namespace planning {

ModelRegistrar::ModelRegistrar()
    : model_factory_(
          std::make_shared<Factory<FollowModelBase, std::string>>()) {
  RegisterAllCarFollowModel();
}

std::shared_ptr<FollowModelBase> ModelRegistrar::CreateProduct(
    const std::string& model_name) {
  return std::move(model_factory_->CreateProduct(model_name));
}

void ModelRegistrar::RegisterAllCarFollowModel() {
  RegisterCarFollowModel<FollowModelIDM>(
      CarFollowModel_Type_Name(CarFollowModel::IDM));
}

}  // namespace planning
}  // namespace apollo