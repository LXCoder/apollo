/******************************************************************************
 * Copyright 2025 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file model_registrar.h
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/tasks/follow_model/base/factory.h"
#include "modules/planning/tasks/follow_model/base/follow_model_base.h"

namespace apollo {
namespace planning {

class ModelRegistrar : public IRegistrar<FollowModelBase> {
 public:
  inline static ModelRegistrar* Instance() {
    static ModelRegistrar s_instance;
    return &s_instance;
  }

  std::shared_ptr<FollowModelBase> CreateProduct(const std::string&) override;

  template <typename ModelType>
  void RegisterCarFollowModel(const std::string& model_name);

 private:
  ModelRegistrar();
  ~ModelRegistrar() = default;
  ModelRegistrar(const ModelRegistrar&) = delete;
  ModelRegistrar& operator=(const ModelRegistrar&) = delete;

 private:
  std::shared_ptr<Factory<FollowModelBase, std::string>> model_factory_;
};

template <typename ModelType>
void ModelRegistrar::RegisterCarFollowModel(const std::string& model_name) {
  REGISTER_MODEL_TO_FACTORY(FollowModelBase, ModelType, model_factory_,
                            model_name);
}

}  // namespace planning
}  // namespace apollo
