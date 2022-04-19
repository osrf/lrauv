#include "lrauv_system_tests/LRAUVController.hh"

#include <signal.h>
#include <unistd.h>
#include <cstring>

#include <ignition/common/Console.hh>

#include "TestConstants.hh"

namespace lrauv_system_tests
{
namespace detail
{
int popen(const char *_cmd, const char *_cwd, FILE **_stdout)
{
  int fd[2];
  if (pipe(fd) == -1)
  {
    perror("pipe");
    return -1;
  }
  int pid = fork();
  if (pid == -1)
  {
    perror("fork");
    return -1;
  }
  if (pid == 0)
  {
    close(fd[0]);
    if (dup2(fd[1], 1) == -1)
    {
      perror("dup2");
      exit(-1);
    }
    if (setpgid(pid, pid) == -1)
    {
      perror("setpgid");
      exit(-1);
    }
    if (chdir(_cwd) == -1)
    {
      perror("chdir");
      exit(-1);
    }
    return execl("/bin/sh", "/bin/sh", "-c", _cmd, NULL);
  }
  close(fd[1]);
  *_stdout = fdopen(fd[0], "r");
  if (*_stdout == nullptr)
  {
    perror("fdopen");
    kill(pid, SIGTERM);
    close(fd[0]);
    return -1;
  }
  return pid;
}
}

LRAUVController LRAUVController::Execute(const std::string &_mission)
{
  std::ostringstream os;
  os << LRAUV_APP_PATH << "/bin/LRAUV"
     << " -c regressionTests/ignitionTests"
     << " -x 'run " << LRAUV_APP_PATH << "/Missions/"
     << _mission << " quitAtEnd' 2>&1";
  const std::string cmd = os.str();
  igndbg << "Running command [" << cmd << "]" << std::endl;
  FILE * stdout = nullptr;
  int pid = detail::popen(cmd.c_str(), LRAUV_APP_PATH, &stdout);
  if (pid == -1)
  {
    ignerr << "Failed to execute command [" << cmd << "]." << std::endl;
    throw std::runtime_error("Failed to execute command [" + cmd + "]");
  }
  return LRAUVController(pid, stdout);
}

LRAUVController::LRAUVController(int _pid, FILE *_stdout) : pid(_pid)
{
  this->backgroundThread = std::thread([_stdout](){
    char buffer[512];
    while (fgets(buffer, 512, _stdout))
    {
      igndbg << "CMD OUTPUT: " << buffer << std::endl;

      if (strstr(buffer, "FAULT") || strstr(buffer, "CRITICAL"))
      {
        ignerr << buffer << std::endl;
      }

      const char * quitMessage =
          "Stop Mission called by Supervisor::terminate";
      if (strstr(buffer, quitMessage))
      {
        ignmsg << "LRAUV controller quit" << std::endl;
        break;
      }
    }
    pclose(_stdout);
  });
}

LRAUVController::~LRAUVController()
{
  this->Kill();
}

bool LRAUVController::IsRunning() const
{
  return kill(this->pid, 0) == 0;
}

void LRAUVController::Kill()
{
  kill(this->pid, SIGINT);
  if (this->backgroundThread.joinable())
  {
    this->backgroundThread.join();
  }
}

}
