#pragma once
#include <fmt/format.h>

#include "exception_base.h"
#include "fmt/format.h"

namespace vamana {

class Exception : public ExceptionBase {
 public:
  Exception() = default;

  Exception(int code, const std::string &msg) : ExceptionBase(msg, code) {}

  template <typename... Args>
  Exception(int code, const std::string &fmt, Args &&...args)
      : ExceptionBase(fmt::format(fmt, std::forward<Args>(args)...), code) {}

  Exception *clone() const override { return new Exception(*this); }

  void rethrow() const override { throw *this; }

  const char *name() const throw() override { return "sql::Exception"; }

  const char *what() const throw() override { return message().data(); }

 private:
  const char *className() const throw() override { return "sql::Exception"; }
};

}  // namespace vamana