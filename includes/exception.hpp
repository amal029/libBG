#include <exception>
#include <string>

class DeletedException : public std::exception {
private:
  std::string message;

public:
  explicit DeletedException(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class NumPorts : public std::exception {
private:
  std::string message;

public:
  explicit NumPorts(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class NumEdges : public std::exception {
private:
  std::string message;

public:
  explicit NumEdges(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class InCorrectComponentConnection : public std::exception {
private:
  std::string message;

public:
  explicit InCorrectComponentConnection(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class PortIndexOutofBounds : public std::exception {
private:
  std::string message;

public:
  explicit PortIndexOutofBounds(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class JunctionContraintViolated : public std::exception {
private:
  std::string message;

public:
  explicit JunctionContraintViolated(const std::string &msg) : message(msg) {}
  const char *what() const noexcept override { return message.c_str(); }
};

class JunctionAssignment : public std::exception {
private:
  const char* message;

public:
  explicit JunctionAssignment(const char *msg) : message(msg) {}
  const char *what() const noexcept override { return message; }
};
