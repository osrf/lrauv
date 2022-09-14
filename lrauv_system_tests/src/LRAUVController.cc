#include "lrauv_system_tests/LRAUVController.hh"

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <system_error>

#include <gz/common/Console.hh>
#include <gz/common/StringUtils.hh>

#include "TestConstants.hh"

namespace lrauv_system_tests
{
namespace detail
{
int popen(const char *_path, char * const _argv[],
          const char *_cwd, FILE **_stdout)
{
  int stdout_pipe[2];
  if (pipe(stdout_pipe) == -1)
  {
    return -1;
  }
  int pid = fork();
  if (pid == -1)
  {
    return -1;
  }
  if (pid == 0)
  {
    close(stdout_pipe[0]);
    int nullfd = open("/dev/null", O_RDONLY);
    if (nullfd == -1)
    {
      perror("open");
      exit(1);
    }
    if (dup2(nullfd, STDIN_FILENO) == -1)
    {
      perror("dup2");
      exit(1);
    }
    if (dup2(stdout_pipe[1], STDOUT_FILENO) == -1)
    {
      perror("dup2");
      exit(1);
    }
    if (dup2(stdout_pipe[1], STDERR_FILENO) == -1)
    {
      perror("dup2");
      exit(1);
    }
    if (chdir(_cwd) == -1)
    {
      perror("chdir");
      exit(1);
    }
    if (execv(_path, _argv) == -1)
    {
      perror("execv");
      exit(1);
    }
    exit(0);
  }
  close(stdout_pipe[1]);
  *_stdout = fdopen(stdout_pipe[0], "r");
  if (!*_stdout)
  {
    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);
    close(stdout_pipe[0]);
    return -1;
  }
  return pid;
}
}

namespace
{
std::vector<char *> BorrowAsArgv(std::vector<std::string> &_cmd)
{
  std::vector<char *> argv;
  argv.reserve(_cmd.size() + 1);
  for (auto &arg : _cmd)
  {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);
  return argv;
}
}

LRAUVController
LRAUVController::Execute(const std::vector<std::string> &_commands)
{
  const char *executable = LRAUV_APP_PATH "/bin/LRAUV";
  std::vector<std::string> cmd = {
    executable, "-c", "regressionTests/gazeboTests"};
  for (const auto &command : _commands)
  {
    cmd.push_back("-x");
    cmd.push_back(command);
  }
  const std::string cmd_string = gz::common::Join(cmd, " ");
  gzdbg << "Running command [" << cmd_string << "]" << std::endl;
  FILE * stdout = nullptr;
  std::vector<char *> argv = BorrowAsArgv(cmd);
  int pid = detail::popen(
      executable, argv.data(), LRAUV_APP_PATH, &stdout);
  if (pid == -1)
  {
    throw std::system_error(
        errno, std::system_category(), "Failed to execute command [" +
        cmd_string + "] due to " + strerror(errno));
  }
  return LRAUVController(pid, stdout);
}

LRAUVController::LRAUVController(int _pid, FILE *_stdout) : pid(_pid)
{
  this->backgroundThread = std::thread([_stdout]()
  {
    char buffer[512];
    while (fgets(buffer, 512, _stdout))
    {
      gzdbg << "CMD OUTPUT: " << buffer;

      if (strstr(buffer, "FAULT") || strstr(buffer, "CRITICAL"))
      {
        gzerr << buffer;
      }
    }
    fclose(_stdout);
  });
}

LRAUVController::~LRAUVController()
{
  try
  {
    if (this->Kill(SIGTERM))
    {
      this->Wait();
    }
  }
  catch(const std::system_error &e)
  {
    gzerr << e.what() << std::endl;
  }
}

bool LRAUVController::IsRunning()
{
  return this->Kill(0);
}

bool LRAUVController::Kill(int signal)
{
  int ret = kill(this->pid, signal);
  if (ret == -1)
  {
    if (errno != ESRCH)
    {
      throw std::system_error(
          errno, std::system_category(),
          strerror(errno));
    }
    return false;
  }
  return true;
}

int LRAUVController::Wait()
{
  int status;
  int ret = waitpid(this->pid, &status, 0);
  if (ret != this->pid)
  {
    throw std::system_error(
        errno, std::system_category(),
        strerror(errno));
  }
  if (this->backgroundThread.joinable())
  {
    this->backgroundThread.join();
  }
  return WIFEXITED(status) ? WEXITSTATUS(status) : WTERMSIG(status);
}

}
