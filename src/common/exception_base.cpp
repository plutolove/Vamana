#include <typeinfo>

#include "exception_base.h"

ExceptionBase::ExceptionBase(int code) : _pNested(0), _code(code) {}

ExceptionBase::ExceptionBase(const std::string& msg, int code)
    : _msg(msg), _pNested(0), _code(code) {}

ExceptionBase::ExceptionBase(const std::string& msg, const std::string& arg,
                             int code)
    : _msg(msg), _pNested(0), _code(code) {
  if (!arg.empty()) {
    _msg.append(": ");
    _msg.append(arg);
  }
}

ExceptionBase::ExceptionBase(const std::string& msg,
                             const ExceptionBase& nested, int code)
    : _msg(msg), _pNested(nested.clone()), _code(code) {}

ExceptionBase::ExceptionBase(const ExceptionBase& exc)
    : std::exception(exc), _msg(exc._msg), _code(exc._code) {
  _pNested = exc._pNested ? exc._pNested->clone() : 0;
}

ExceptionBase::~ExceptionBase() throw() { delete _pNested; }

ExceptionBase& ExceptionBase::operator=(const ExceptionBase& exc) {
  if (&exc != this) {
    ExceptionBase* newPNested = exc._pNested ? exc._pNested->clone() : 0;
    delete _pNested;
    _msg = exc._msg;
    _pNested = newPNested;
    _code = exc._code;
  }
  return *this;
}

const char* ExceptionBase::name() const throw() { return "ExceptionBase"; }

const char* ExceptionBase::className() const throw() {
  return typeid(*this).name();
}

const char* ExceptionBase::what() const throw() { return name(); }

std::string ExceptionBase::displayText() const {
  std::string txt = name();
  if (!_msg.empty()) {
    txt.append(": ");
    txt.append(_msg);
  }
  return txt;
}

void ExceptionBase::extendedMessage(const std::string& arg) {
  if (!arg.empty()) {
    if (!_msg.empty()) _msg.append(": ");
    _msg.append(arg);
  }
}

ExceptionBase* ExceptionBase::clone() const { return new ExceptionBase(*this); }

void ExceptionBase::rethrow() const { throw *this; }
