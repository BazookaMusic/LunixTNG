#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <sys/ioctl.h>

int main(int argc, char ** argv)
{
	if(argc != 4)
		printf("Usage: ./lunix-test SENSOR IOCTL BLOCKING\n");
	else
	{
		int input_fd;
        int cmd;
		int readFlags;

        int p;

        if(argv[2][0] == '0')
            cmd = 0;
        else
            cmd = 1;

        if(argv[3][0] == '0')
            readFlags = 0;
        else
            readFlags = O_NONBLOCK;
	
		if(argv[1][0] == 't') 
            input_fd = open("/dev/lunix0-temp", readFlags);
        else if(argv[1][0] == 'b')
            input_fd = open("/dev/lunix0-batt", readFlags);
        else
            input_fd = open("/dev/lunix0-light", readFlags);

        if (input_fd == -1)
		{
			perror("open");
			exit(1);
		}
		else
		{
            p = fork();

            if(p == 0 && ioctl(input_fd, cmd) == -EINVAL)
            {
                perror("ioctl");
            }

			char buff[20];
            ssize_t read_count;
            while(1)
            {
                    read_count = read(input_fd, buff, sizeof(buff));
                    if(read_count == 0)
                            break;
                    else if(read_count == -1)
                    {
                            perror("read");
                            break;
                    }
                    buff[read_count] = '\0';
                    
                    if(p == 0)
                        printf("Child: %s", buff);
                    else
                        printf("Parent: %s", buff);
            }
            close(input_fd);
        }
	}
	return 0;
}
