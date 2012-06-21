
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <errno.h>

void run_gams_on_fname(char *buf, int sock);

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

int main(int argc, char *argv[])
{
  int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[1024];
  struct sockaddr_in serv_addr, cli_addr;
  int n;

  signal(SIGCHLD, SIG_IGN);

  //Create a Socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");

  //server address
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = 20202;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);

  //bind to socket to the address
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  if (listen(sockfd, 5) == -1)
    error("Listening failed");

  clilen = sizeof(cli_addr);

  while(1) {
    printf("Waiting for client to contact...\n");
    newsockfd = accept(sockfd,
                       (struct sockaddr *) &cli_addr,
                       &clilen);
    if (newsockfd < 0) {
      fprintf(stderr, "ERROR on accept\n");
      continue;
    }
    bzero(buffer,1024);
    n = read(newsockfd, buffer, 1024);
    if (n < 0) {
      fprintf(stderr, "ERROR reading from socket\n");
      continue;
    }
    if (strcmp(buffer, "__END__") == 0) {
      const char *CMD = "__DONE__";
      int n = write(newsockfd, CMD, strlen(CMD));
      if (n < 0)
        fprintf(stderr, "Error writing to socket\n");
      close(newsockfd);
      break;
    }
    if (strcmp(buffer, "run-gams") != 0) {
      fprintf(stderr, "Wrong command\n");
      const char *CMD = "__ERR__";
      int n = write(newsockfd, CMD, strlen(CMD));
      if (n < 0)
        fprintf(stderr, "Error writing to socket\n");
      close(newsockfd);

      continue;
    }

    bzero(buffer,1024);
    n = read(newsockfd, buffer, 1023);
    if (n < 0) {
      fprintf(stderr, "ERROR reading from socket\n");
      continue;
    }
    fprintf(stderr, "Running gams on %s\n", buffer);

    run_gams_on_fname(buffer, newsockfd);

  }
  fprintf(stderr, "I am done...\n");
  close(sockfd);
  return 0;
}

void run_gams_on_fname(char *buf, int sock)
{
  int childpid  = fork();

  if (childpid == -1) {
    fprintf(stderr, " Error forking the child %d\n", errno);
    close(sock);
    return;
  }
  if (childpid != 0) {
    //parent
    close(sock);
    return;
  }
  // run gams
  const char *dname = dirname(strdup(buf));
  const char *fname = basename(buf);

  char cmd_buf[1024];
  const char *gams_path = "/p/gams/systems/lnx_64/23.6/gams";
  sprintf(cmd_buf, "cd %s && %s %s >gams.log", dname, gams_path, fname);
  sprintf(cmd_buf, "cd %s && gams %s >gams.log", dname, fname);

  printf("running %s\n", cmd_buf);
  system(cmd_buf);

  const char *CMD = "__DONE__";
  int n = write(sock, CMD, strlen(CMD));
  if (n < 0)
    fprintf(stderr, "Error wring to socket\n");

  close(sock);
  exit(0);
}
