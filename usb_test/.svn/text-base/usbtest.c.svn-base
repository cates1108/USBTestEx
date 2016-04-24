#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>//TCC110729 => http://lists.freebsd.org/pipermail/freebsd-standards/2003-July/000171.html
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <time.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define CP_MSG 0
#define RBUFSIZE 1048576     //1024*1024 = 1MB
#define WBUFSIZE 1048576     //1024*1024 = 1MB
#define CAL_TIME(a,b) (a + b / 1000.0 /1000.0 /1000.0)

/* Global Variable */
int loop = 1, diff_flag = 1;
struct stat stat_src;
FILE *fp_src, *fp_des;
char* gSrc_path = NULL;
char* gDes_path = NULL;

/* Static Variable */
static struct timespec gPre_time, gPost_time;

struct buffer
{
    void   *start;
    size_t length;
};

typedef enum eSwitchType
{
    SWITCH_ON = 1,
    SWITCH_OFF

}SwitchType;

int _comparefile(void)
{
	int i = 0;
	FILE *f1, *f2;
	int c1, c2;
	int ret = 1;

	f1 = fopen(gSrc_path, "r");
	if(!f1){
		printf("open file %s failed!", gSrc_path);
		goto open_f1_failed;
	}

	f2 = fopen(gDes_path, "r");
	if(!f2){
		printf("open file %s failed!", gDes_path);
		goto open_f2_failed;
	}

	c1 = fgetc(f1);
	c2 = fgetc(f2);

    while((c1!=EOF)&&(c2!=EOF))
    {
 		if (c1!=c2) {
			char buf[80];
			char *ptr = buf;

			i = ftell(f1);
			printf("Error!! different!!\t\t(%02x,%02x) @ 0x%x\n", c1, c2, i);

			fclose(f1);
			fclose(f2);

			//sleep(1);
			//remount_fs("/dev/sda", "/sda");

			f1 = fopen(gSrc_path, "r");
			f2 = fopen(gDes_path, "r");

			printf("  ******** original data ********         ********  copied data  ********\n");

			i = (i<64) ? 0 : i-64;
			fseek(f1, i, SEEK_SET);
			fseek(f2, i, SEEK_SET);

			memset(buf, ' ', sizeof(buf)-2);
			buf[sizeof(buf)-2] = '\n';
			buf[sizeof(buf)-1] = '\0';
			ptr += (ftell(f1)%16)*2;

			while( (c1 = fgetc(f1)) != EOF && (c2 = fgetc(f2)) != EOF )
			{
				sprintf(ptr, "%02x", c1);
				sprintf(ptr+40, "%02x", c2);
				ptr += 2;

				if(!(ftell(f1) % 16))
				{
					*ptr = ' ';	// replace '\0'
					*(ptr + 40) = '\n';
					*(ptr + 41) = '\0';
					printf("%s", buf);
					ptr = buf;
					memset(buf, ' ', sizeof(buf)-2);
				}
				else if(!(ftell(f1) % 4))
				{
					*(ptr) = ' ';
					*(ptr+40) = ' ';
					++ptr;
				}
				if(ftell(f1) - i >= 16 * 10)
				{
					*(ptr) = ' ';
					*(ptr+40) = ' ';
					printf("%s\n", buf);
					break;
				}
			}

			printf("\n");

			goto out;
		}
		c1=fgetc(f1);
		c2=fgetc(f2);
	}

	if( (c1==EOF) && (c2==EOF) ) {
		printf("########################################\n");
		printf("OK!! File is the same with size %ld~\n", ftell(f1));
		printf("########################################\n");
	}
  	else {
		printf("############################################################################\n");
		printf("Error!! file sizes are different!!\t\t(%02x,%02x) @ %ld\n", c1, c2, ftell(f1));
		printf("############################################################################\n");
		goto out;
	}

	ret = 0;

out:
	fclose(f2);
open_f2_failed:
	fclose(f1);
open_f1_failed:
	return ret;

}

void _timerswitch(SwitchType OnOff)
{
    if(OnOff == SWITCH_ON)
        clock_gettime(CLOCK_MONOTONIC, &gPre_time);
    else
        clock_gettime(CLOCK_MONOTONIC, &gPost_time);
}

struct timespec _gettime(void)
{
    struct timespec temp_time;

    memset(&temp_time, 0, sizeof(struct timespec));

    temp_time.tv_sec = gPost_time.tv_sec  - gPre_time.tv_sec;
    temp_time.tv_nsec = gPost_time.tv_nsec - gPre_time.tv_nsec;

    // reset time
    memset(&gPre_time, 0, sizeof(struct timespec));
    memset(&gPost_time, 0, sizeof(struct timespec));

    return temp_time;
}

void AP_USB_Copytest(void)
{
	int i, ok = 0;
	char *buf, *pbuf;
	int read_size, ret, per_loop_read_size, total_read_size=0;
	double final_total_sec=0;
	double USB_speed=0;
    struct timespec diff_time, total_time;

    memset(&total_time, 0, sizeof(struct timespec));

	for(i = (loop) ? loop : ~0; i > 0; --i)
	{
		per_loop_read_size=0;
		printf("%dth testing ...........................................\n",loop-i+1);

	    /* buffer */
		buf = malloc(RBUFSIZE);

		if( buf == NULL )
        {
			fprintf( stderr, "ERROR!! allocate memory(%dM) failed! (#%d: %s)\n",1048576/1048576, errno, strerror(errno) );
			exit(1);
		}


	    /* read and write through pbuf */
		pbuf = buf;

        _timerswitch(SWITCH_ON);
        while( !feof(fp_src) )
		{
			ret = fread( pbuf, 1, RBUFSIZE, fp_src );

			if( ret <  RBUFSIZE )
            {
				if( !feof(fp_src) )
                {
					fprintf( stderr, "ERROR!! read SRC failed! (#%d: %s)\n", errno, strerror(errno) );
					exit(1);
				}
			}
			read_size = ret;
			per_loop_read_size += read_size;
			total_read_size += read_size;

            printf("read_size=%d, per_loop_read_size=%d, total_read_size=%d\n", read_size, per_loop_read_size, total_read_size);

			ret = fwrite( pbuf, 1, read_size, fp_des );

			if( ret < read_size )
			{
				fprintf( stderr, "ERROR!! write DES failed! (#%d: %s)\n", errno, strerror(errno) );
				exit(1);
			}
		}

        fflush(fp_des); //clear stdin register
		fsync(fileno(fp_des));  //sync buff to disk
        _timerswitch(SWITCH_OFF);

        diff_time = _gettime();
		total_time.tv_sec = total_time.tv_sec + diff_time.tv_sec;
		total_time.tv_nsec = total_time.tv_nsec + diff_time.tv_nsec;


        //printf("finished %dth................. ok/total %d/%d .......................................\n\n",loop-i+1, loop, loop-i+1);

        if((loop-i+1)==loop)
    	{
    	    final_total_sec = CAL_TIME((double)total_time.tv_sec,(double)total_time.tv_nsec)/loop;
            printf("AVG R/W TOTAL TIME:%f sec\n",final_total_sec);
        	USB_speed = ((stat_src.st_size) /final_total_sec)/(1024*1024);
        	printf("SPEED:%f MBps\n",USB_speed);
        }

		free(buf);

	   /* execute diff mechanism */
	   if(diff_flag)
       {
			printf("call diff\n");
			if( !_comparefile() )
			   ++ok;
			else
			{
				printf("File Compare error and finished and exit...\n");
				exit(1);
			}

			printf("File Compare finished...\n");
       }/*if(diff_flag)*/


		sleep(1);
        printf("File Copy finished...\n"); //---> [GENE]
		printf("finished %dth................. ok/total %d/%d .......................................\n",loop-i+1, loop, loop-i+1);

        fseek(fp_src,0,SEEK_SET);
        fseek(fp_des,0,SEEK_SET);

	}/*TCC110729: for(i = (loop) ? loop : ~0; i > 0; --i)*/

	fclose(fp_src);
	fclose(fp_des);
}

void AP_USB_SpeedTest(void)
{
    int i;
	char *rbuf = NULL, *wbuf = NULL;
	int read_size, ret, total_read_size=0;
	double final_total_sec=0;
	float USB_speed=0;
    struct timespec diff_time, total_time;

	/* buffer */
	rbuf = malloc(RBUFSIZE);
	wbuf = malloc(WBUFSIZE);

    printf("============Buffer Size:%d MB===========\n", RBUFSIZE/(1024*1024));

    if( rbuf == NULL )
    {
	    fprintf( stderr, "ERROR!! allocate memory(%dM) failed! (#%d: %s)\n",1048576/1048576, errno, strerror(errno) );
		exit(1);
	}

    memset(&total_time, 0, sizeof(struct timespec));

    printf("===============Start Read Test=================\n");
	for(i = (loop) ? loop : ~0; i > 0; --i)
	{

	    /* read file through buf */
        _timerswitch(SWITCH_ON);
		while( !feof(fp_src) )
		{
			ret = fread( rbuf, 1, RBUFSIZE, fp_src );

			if( ret < RBUFSIZE )
            {
				if( !feof(fp_src) )
                {
					fprintf( stderr, "ERROR!! read SRC failed! (#%d: %s)\n", errno, strerror(errno) );
					exit(1);
				}
			}
			read_size = ret;
			total_read_size += read_size;

            //printf("read_size=%d, total_read_size=%d\n", read_size, total_read_size);
		}
        _timerswitch(SWITCH_OFF);

        diff_time = _gettime();
		total_time.tv_sec = total_time.tv_sec + diff_time.tv_sec;
		total_time.tv_nsec = total_time.tv_nsec + diff_time.tv_nsec;

        printf("finished %d/%d : %f sec......OK \n",loop, loop-i+1, CAL_TIME((double)diff_time.tv_sec,(double)diff_time.tv_nsec));

        if((loop-i+1)==loop)
        {
            final_total_sec = CAL_TIME((double)total_time.tv_sec,(double)total_time.tv_nsec)/loop;
            printf("AVG R TOTAL TIME:%f sec\n",final_total_sec);

        	USB_speed = ((stat_src.st_size) /final_total_sec)/(1024*1024);
        	printf("SPEED:%f MBps\n",USB_speed);
        }

        fflush(fp_src); //clear stdin register
        fseek(fp_src,0,SEEK_SET);

    } //loopo end


	/* write file through buf */
    srand((int) time(0));
    for (i = 0; i < WBUFSIZE; i++)
    {
        wbuf[i] = rand()%256;
    }

    memset(&total_time, 0, sizeof(struct timespec));

    printf("\n===============Start Write Test %dMB=================\n",WBUFSIZE/(1024*1024));
	for(i = (loop) ? loop : ~0; i > 0; --i)
    {

        _timerswitch(SWITCH_ON);
        ret = fwrite( wbuf, 1, WBUFSIZE, fp_des);

        if( ret < WBUFSIZE)
    	{
    		printf("ERROR!! write %s failed! (#%d: %s)\n", gDes_path, errno, strerror(errno) );
    		exit(1);
    	}

        fflush(fp_des); //clear stdin register
		fsync(fileno(fp_des));  //sync buff to disk

        _timerswitch(SWITCH_OFF);

        diff_time = _gettime();
		total_time.tv_sec = total_time.tv_sec + diff_time.tv_sec;
		total_time.tv_nsec = total_time.tv_nsec + diff_time.tv_nsec;

        printf("finished %d/%d : %f sec......OK \n",loop, loop-i+1, CAL_TIME((double)diff_time.tv_sec,(double)diff_time.tv_nsec));

        if((loop-i+1)==loop)
        {
        	final_total_sec = CAL_TIME((double)total_time.tv_sec,(double)total_time.tv_nsec)/loop;
            printf("AVG W TOTAL TIME:%f sec\n",final_total_sec);

        	USB_speed = (WBUFSIZE/final_total_sec)/(1024*1024);//MB per second
        	printf("SPEED:%f MBps\n",USB_speed);//TCC110915
        }

		fseek(fp_des,0,SEEK_SET);
    }

    fclose(fp_src);
    fclose(fp_des);
    free(rbuf);
    free(wbuf);

	sleep(1);
    printf("USB Speed finished...\n\n");
}

void AP_USB_MouseTest(unsigned char mousenum)
{
	int fp = -1,rp=-1;
	char mousename[16],buffer[3];
	char x=0,y=0,pre_x=0,pre_y=0,lb=0,rb=0,mb=0;	//x, y movement, left/right/mid button flag

	sprintf(mousename,"/dev/mouse%d",mousenum);
    fp = open(mousename, O_RDONLY,0);

    if(fp<0)
	{
		printf("open mouse%d failed\n",mousenum);
		return;
	}
	printf("If you want to exit mouse test, please press left/mid/right button\n");
    	while(1)
	{
    		rp = read(fp, buffer, 3);
		if(rp<0)
		{
			printf("read mouse%d  failed\n",mousenum);
			break;
		}
		x=buffer[1];			//Byte 2  X movement
		y=buffer[2];			//Byte 3  Y movement
		lb=(buffer[0]&1) >0;	//Byte 1  Y overflow | X overflow | Y sign bit | X sign bit | Always 1 | Middle Btn | Right Btn | Left Btn
		rb=(buffer[0]&2) >0;
		mb=(buffer[0]&4) >0;

		if((pre_x!=x)||(pre_y!=y))
			printf("mouse%d movement x : %d | y : %d \n",mousenum, x, y);

		if(lb)
		{
			printf("mouse%d left button pressed\n",mousenum);
			break;
		}else if (mb){
			printf("mouse%d mid button pressed\n",mousenum);
			break;
		}else if(rb){
			printf("mouse%d right button pressed\n",mousenum);
			break;
		}
		pre_x=x;
		pre_y=y;
    	}
	close(fp);
    	printf("Mouse Test finished...\n");
}

void AP_USB_KeycodeTest(unsigned char eventnum)
{
	int fp=-1,rp=-1;
	char eventname[16];
	struct input_event ev;

	sprintf(eventname,"/dev/event%d",eventnum);

	fp=open(eventname,O_RDONLY, 0);
	if(fp<0) {
		printf("open keyboard%d failed\n",eventnum);
		return;
	}
	printf("If you want to exit keycode test, please press ESC button\n");
	while(1)
	{
		rp=read(fp,&ev,sizeof(ev));
		if(rp<0)
		{
			printf("read keyboard%d  failed\n",eventnum);
			break;
		}

		if(ev.type == EV_KEY && ev.value==1) //value:1 (press buttom) 0: (release button)
		{
			printf("keyboard%d pressed code=%d\n",eventnum,ev.code);
			if(ev.code==KEY_ESC)
			{
				printf("ESC pressed & exit keycode%d test\n",eventnum);
				break;
			}
		}
	}
	close(fp);
	printf("Keycode Test finished...\n");
}

static int xioctl(int fd, int request, void * arg)
{
	int r;

       do {
       	r = ioctl(fd, request, arg);
       } while (-1 == r && EINTR == errno);

	return r;
}

void AP_USB_CaptureTest(unsigned char picnum)
{
	struct v4l2_format              fmt;
       struct v4l2_buffer              buf;
       struct v4l2_requestbuffers      req;
       enum v4l2_buf_type              type;
       fd_set                          fds;
       struct timeval                  tv;
       int                             r, fd = -1;
       unsigned int                    i, n_buffers;
       char                            *dev_name = "/dev/video0";
       char                            *out_name="test.yuv\0";
       FILE                            *fout;
       struct buffer                   *buffers;
	unsigned int min;

	if(picnum<1 || picnum>30)
	{
		printf("[Liang] pic num is invalid. The valid range is from 1 to 30\n.");
		return;
	}

	fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
       if (fd < 0) {
                printf("[Liang] Cannot open device\n");
                return;
        }

	// set format in
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640;
        fmt.fmt.pix.height      = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
  	if(xioctl(fd, VIDIOC_S_FMT, &fmt)==-1)
       	printf("[Liang] xioctl ret fail1\n");

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	// request buffers
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
	if(xioctl(fd, VIDIOC_REQBUFS, &req)==-1)
		printf("[Liang] xioctl ret fail2\n");

	if (req.count < 2)
		printf("[Liang] Insufficient buffer memory\n");

       buffers = malloc(req.count*sizeof(*buffers));

	if(!buffers)
		printf("[Liang] Out of memory\n");


	// map the buffers
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

              if(xioctl(fd, VIDIOC_QUERYBUF, &buf)==-1)
			printf("[Liang] xioctl ret fail3\n");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start = mmap(NULL, buf.length,
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start) {
                        printf("[Liang] mmap\n");
                }
        }

	// Queue the buffers
        for (i = 0; i < n_buffers; ++i) {
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
              if(xioctl(fd, VIDIOC_QBUF, &buf)==-1)
	       	printf("[Liang] xioctl ret fail4\n");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if(xioctl(fd, VIDIOC_STREAMON, &type)==-1)
		printf("[Liang] xioctl ret fail5\n");

       fout = fopen(out_name, "wa+");
       if (!fout)
       	printf("[Liang] fopen out_name fail\n");

	for (i = 0; i < picnum; i++) {
         	do {
               	FD_ZERO(&fds);
              	FD_SET(fd, &fds);

                 	/* Timeout. */
                  	tv.tv_sec = 2;
             		tv.tv_usec = 0;

              	r = select(fd + 1, &fds, NULL, NULL, &tv);
              } while ((r == -1 && (errno = EINTR)));

		if (r == -1)
              	printf("[Liang] select\n");

		if(r == 0)
			printf("[Liang] select timeout\n");

              CLEAR(buf);
              buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
              buf.memory = V4L2_MEMORY_MMAP;
              if(xioctl(fd, VIDIOC_DQBUF, &buf)==-1)
			printf("[Liang] xioctl ret fail6\n");

		assert(buf.index < n_buffers);
              fwrite(buffers[buf.index].start, buf.length, 1, fout);
		if(xioctl(fd, VIDIOC_QBUF, &buf)==-1)
			printf("[Liang] xioctl ret fail7\n");
       }
	fclose(fout);

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
       if(xioctl(fd, VIDIOC_STREAMOFF, &type)==-1)
		printf("[Liang] xioctl ret fail8\n");

       for (i = 0; i < n_buffers; ++i)
		if(munmap(buffers[i].start, buffers[i].length)==-1)
              	printf("[Liang] munmap fail\n");

	free(buffers);

	if(close(fd)==-1)
	{
		printf("[Liang] close fail\n");
		fd=-1;
	}
	return;

}

int main(int argc, char* argv[])
{

	unsigned char dev_num=0;
	// Copy test and Speed test arg
    if(argc == 5)
    {
        gSrc_path = argv[2];
        gDes_path = argv[3];
    	loop = atoi(argv[4]);
        //umount_flag = atoi(argv[4]);//TCC110729
        //test_item = atoi(argv[5]);
        //diff_flag = atoi(argv[5]);//TCC110729

        if((gDes_path == NULL)||( gSrc_path == NULL))
        {
        	printf("ERROR!! open %s failed! (#%d: %s)", gDes_path, errno, strerror(errno));
        	exit(1);
        }

        /*read source */
    	fp_src = fopen(gSrc_path, "r");
    	if( fp_src == NULL)
        {
    		fprintf( stderr, "ERROR!! open %s failed! (#%d: %s)", gSrc_path, errno, strerror(errno));
    		exit(1);
    	}

        fstat(fileno(fp_src), &stat_src);

    	/*write destination */
    	fp_des = fopen(gDes_path, "w+");

    	if( fp_des == NULL)
        {
    	    fprintf( stderr, "ERROR!! open %s failed! (#%d: %s)", gDes_path, errno, strerror(errno));
    		exit(1);
    	}

        //setvbuf ( fp_src , NULL , _IONBF , 0 );
        //setvbuf ( fp_des , NULL , _IONBF , 0 );

    }
    else if(argc == 3)
    {
		dev_num = atoi(argv[2]);
    }
    else
    {
		printf( "EX: ./usbtest copy 1:SRC 2:DES 3:loop\n" );
		printf( "EX: ./usbtest speed 1:SRC 2:DES 3:loop\n" );
		printf( "EX: ./usbtest mouse 1:mouse num\n");
        printf( "EX: ./usbtest keycode 1:event num\n");
        printf( "EX: ./usbtest capture\n");
		exit(1);
    }

    if(strcmp(argv[1],"copy")==0 || strcmp(argv[1],"COPY")==0)
    {
        printf("Start Copy Test!!\n");
        AP_USB_Copytest();
        unlink(gDes_path);
    }
    else if(strcmp(argv[1],"speed")==0 || strcmp(argv[1],"SPEED")==0)
    {
        printf("Start Speed Test!!\n");
        AP_USB_SpeedTest();
        unlink(gDes_path);
    }
    else if(strcmp(argv[1],"mouse")==0 || strcmp(argv[1],"MOUSE")==0)
    {
        printf("Start Mouse Test!!\n");
	 	AP_USB_MouseTest(dev_num);
    }
    else if(strcmp(argv[1],"keycode")==0 || strcmp(argv[1],"KEYCODE")==0)
    {
        printf("Start Keycode Test!!\n");
	 	AP_USB_KeycodeTest(dev_num);
    }
    else if(strcmp(argv[1],"capture")==0 || strcmp(argv[1],"CAPTURE")==0)
    {
        printf("Start Capture Test!!\n");
		AP_USB_CaptureTest(dev_num);
    }
    else
    {
        printf("No This Test!!\n");
    }

    return 0;
}


