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
 * @file follow_model_factory.h
 **/

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

#define REGISTER_MODEL_TO_FACTORY(BaseType, DerivedType, key) \
  ModelFactory<BaseType>::Instance()->RegisterClass(          \
      key, []() -> std::shared_ptr<BaseType> {                \
        return std::make_shared<DerivedType>();               \
      })

namespace apollo {
namespace planning {

template <typename BaseType, typename KeyType = std::string>
class ModelFactory {
 public:
  using Creator = std::function<std::shared_ptr<BaseType>()>;

  inline static ModelFactory* Instance() {
    static ModelFactory s_instance;
    return &s_instance;
  }

  bool RegisterClass(const KeyType& key, Creator creator) {
    std::lock_guard<std::mutex> lock(mutex_);
    return creators_.emplace(key, std::move(creator)).second;
  }

  std::shared_ptr<BaseType> CreateModel(const KeyType& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = creators_.find(key);
    if (it != creators_.end()) {
      return (it->second)();
    }
    return nullptr;
  }

 private:
  ModelFactory() = default;
  ~ModelFactory() = default;
  ModelFactory(const ModelFactory&) = delete;
  ModelFactory& operator=(const ModelFactory&) = delete;

 private:
  std::unordered_map<KeyType, Creator> creators_;
  std::mutex mutex_;
};

}  // namespace planning
}  // namespace apollo