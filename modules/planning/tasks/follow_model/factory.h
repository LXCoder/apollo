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
 * @file factory.h
 **/

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#define REGISTER_MODEL_TO_FACTORY(BaseType, DerivedType, factory, key) \
  factory->RegisterClass(key, []() -> std::shared_ptr<BaseType> {      \
    return std::make_shared<DerivedType>();                            \
  })

namespace apollo {
namespace planning {

template <typename BaseType, typename KeyType = std::string>
class Factory {
 public:
  using Creator = std::function<std::shared_ptr<BaseType>()>;

  Factory() = default;
  ~Factory() = default;

  bool RegisterClass(const KeyType& key, Creator creator) {
    return creators_.emplace(key, std::move(creator)).second;
  }

  std::shared_ptr<BaseType> CreateProduct(const KeyType& key) {
    auto it = creators_.find(key);
    if (it != creators_.end()) {
      return (it->second)();
    }
    return nullptr;
  }

 private:
  std::unordered_map<KeyType, Creator> creators_;
};

template <class ProductType>
class IRegistrar {
 public:
  // 获取产品对象抽象接口
  virtual std::shared_ptr<ProductType> CreateProduct(const std::string&) = 0;

 protected:
  IRegistrar() {}
  virtual ~IRegistrar() {}

 private:
  // 禁止外部拷贝和赋值操作
  IRegistrar(const IRegistrar&);
  const IRegistrar& operator=(const IRegistrar&);
};

}  // namespace planning
}  // namespace apollo