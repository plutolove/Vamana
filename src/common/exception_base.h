
#include <stdexcept>
/// This is the base class for all exceptions defined
/// in the Poco class library.
class ExceptionBase : public std::exception {
 public:
  ExceptionBase(const std::string& msg, int code = 0);
  /// Creates an exception.

  ExceptionBase(const std::string& msg, const std::string& arg, int code = 0);
  /// Creates an exception.

  ExceptionBase(const std::string& msg, const ExceptionBase& nested,
                int code = 0);
  /// Creates an exception and stores a clone
  /// of the nested exception.

  ExceptionBase(const ExceptionBase& exc);
  /// Copy constructor.

  ~ExceptionBase() throw();
  /// Destroys the exception and deletes the nested exception.

  ExceptionBase& operator=(const ExceptionBase& exc);
  /// Assignment operator.

  virtual const char* name() const throw();
  /// Returns a static string describing the exception.

  virtual const char* className() const throw();
  /// Returns the name of the exception class.

  virtual const char* what() const throw();
  /// Returns a static string describing the exception.
  ///
  /// Same as name(), but for compatibility with std::exception.

  const ExceptionBase* nested() const;
  /// Returns a pointer to the nested exception, or
  /// null if no nested exception exists.

  const std::string& message() const;
  /// Returns the message text.

  int code() const;
  /// Returns the exception code if defined.

  std::string displayText() const;
  /// Returns a string consisting of the
  /// message name and the message text.

  virtual ExceptionBase* clone() const;
  /// Creates an exact copy of the exception.
  ///
  /// The copy can later be thrown again by
  /// invoking rethrow() on it.

  virtual void rethrow() const;
  /// (Re)Throws the exception.
  ///
  /// This is useful for temporarily storing a
  /// copy of an exception (see clone()), then
  /// throwing it again.

 protected:
  ExceptionBase(int code = 0);
  /// Standard constructor.

  void message(const std::string& msg);
  /// Sets the message for the exception.

  void extendedMessage(const std::string& arg);
  /// Sets the extended message for the exception.

 private:
  std::string _msg;
  ExceptionBase* _pNested;
  int _code;
};

//
// inlines
//
inline const ExceptionBase* ExceptionBase::nested() const { return _pNested; }

inline const std::string& ExceptionBase::message() const { return _msg; }

inline void ExceptionBase::message(const std::string& msg) { _msg = msg; }

inline int ExceptionBase::code() const { return _code; }