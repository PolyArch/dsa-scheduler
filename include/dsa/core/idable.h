#pragma once

#define DEFATTR(ty, name, value)        \
 private:                               \
  ty name##_{value};                    \
 public:                                \
  const ty &name() { return name##_; }  \
  const ty &name(const ty &new_value) { \
    name##_ = new_value;                \
    return name##_;                     \
  }

namespace dsa {
namespace core {

/*!
 * \brief The very base class that every object has an ID.
 */
class IDAble {
  IDAble(int i) : id_(i) {}
  /* \brief The ID of this object. */
  DEFATTR(int, id, -1)
};

} // namespace core
} // namespace dsa
