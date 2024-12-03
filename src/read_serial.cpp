#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 
#include <assert.h>
#include <errno.h> 
#include <termios.h> 
#include <unistd.h> 
#include <sys/ioctl.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <bitset>
#include <sstream>
using namespace cv;
using namespace std;
//smb://10.20.162.227/ ora media
//cd ~/opt/tutorial/arai/arduino
Point pos(-10,-10);//first position
Mat image(500, 500, CV_8UC3);//y,x

int key;
int c = 500;
int p = 0;      //position number
int nv = 4;		//number of vertices
int cen = 0;

	uint8_t g[10];
	uint16_t bset1 [1];
	uint16_t bset2 [1];
	uint16_t bset3 [1];
	uint16_t bset4 [1];
	uint16_t bset5 [1];
	uint16_t bset12 [1];
	uint16_t bset22 [1];
	uint16_t bset32 [1];
	uint16_t bset42 [1];
	uint16_t bset52 [1];
	uint16_t min1 [1];
	uint16_t min2 [1];
	uint16_t min3 [1];
	uint16_t min4 [1];
	uint16_t min5 [1];
	int i;
	int va = 0;
	int oneces = 0;	
	int min13;
	int max1;
	int flag;	
	
	uint8_t bytesWaiting;
// do something with data
/*
void drowcircle(uint16_t data2){		//drowcircle(bset1[0]);	
	uint16_t r = data2;	//radius
	int d = 0;		//diameter
	//if(r < c){c = r;}
	d = ((r/150)*20);
	if(p == 0 && cen == 1)(d-160);
	//cout << "(a:"<< p << "(b:"<< c <<endl;
		
	if(cen == 0){				
		circle(image, Point(250,250), d, cv::Scalar(255, 0,0),1);
		cen++;
	}
	else if(cen != 0){
 		pos = Point((cos(CV_PI * (2.0*p/nv - 0.5)) * 140 + 250), (sin(CV_PI * (2.0*p/nv - 0.5)) * 140 + 250));				
		circle(image, pos, d, cv::Scalar(255, 0,0),1);
		if(p == 3)imshow("img",image);
		p++;
	}
	
	if(p == nv){p = 0; cen = 0;}
	key = waitKey(10);//cout << "("<< p <<";"<<d<<")"<<endl;
	//sleep(3);
}
*/

int main(){

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    int serial_port = open("/dev/ttyACM0", O_RDWR);
    //ls -l /dev/serial/by-id/
    //check and change port-id
    //sudo chmod 666 /dev/(port-id)
    //get aythority

    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;   
    tty.c_cc[VMIN] = 0;
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    // ----Allocate memory for read buffer, set size according to your needs
    
   //------------------------------------output read

	
	Point pos(-10,-10);
	memset(&bset12, '\0', sizeof(bset1));//fill in the array
	memset(&bset22, '\0', sizeof(bset2));
	memset(&bset32, '\0', sizeof(bset3));
	memset(&bset42, '\0', sizeof(bset4));
	memset(&bset52, '\0', sizeof(bset5));
	
	int countup = 0;
	
	
	
	
	while(1){
 		ioctl(serial_port, FIONREAD, &bytesWaiting);	//read data from serial

		if (bytesWaiting > 25){
			uint8_t read_buf [bytesWaiting+1];
		  	memset(&read_buf, '\0', sizeof(read_buf));
		  	read(serial_port, &read_buf, sizeof(read_buf));
							
			//printf("2bsize(%d\n", bytesWaiting);		
			//printf("3r_bsz(%ld\n", sizeof(read_buf));
			//cout << "6r_bBy(" << std::bitset<16>{read_buf[0]} << endl;
			
			for(i = 0; i < sizeof(read_buf); i++){//find the head point
				//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
				if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
				 	break;
				 	} 
			 	}
			i+=2;	
			//printf("7point(%d\n", i);
			//---------------------------------------------------
			memset(&g, '\0', sizeof(g));
			memcpy(&g, &read_buf[i], sizeof(g));
			//for(int l = 0; l < sizeof(g); l++) cout << "|gdata(" <<  std::bitset<8>{g[l]} << endl;//data check
			
			memset(&bset1, '\0', sizeof(bset1));//fill in the array
			memset(&bset2, '\0', sizeof(bset2));
			memset(&bset3, '\0', sizeof(bset3));
			memset(&bset4, '\0', sizeof(bset4));
			memset(&bset5, '\0', sizeof(bset5));
			
			memcpy(&g, &read_buf[i], sizeof(g));
			memcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << 8;memcpy(&bset1, &g[1], 1);//read buff
			memcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << 8;memcpy(&bset2, &g[3], 1);
			memcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << 8;memcpy(&bset3, &g[5], 1);
			memcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << 8;memcpy(&bset4, &g[7], 1);
			memcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << 8;memcpy(&bset5, &g[9], 1);
			
			
		
			
			while(oneces == 0){
				ioctl(serial_port, FIONREAD, &bytesWaiting);	//read data from serial

				if (bytesWaiting > 25){
					uint8_t read_buf [bytesWaiting+1];
				  	memset(&read_buf, '\0', sizeof(read_buf));
				  	read(serial_port, &read_buf, sizeof(read_buf));
									
					//printf("2bsize(%d\n", bytesWaiting);		
					//printf("3r_bsz(%ld\n", sizeof(read_buf));
					//cout << "6r_bBy(" << std::bitset<16>{read_buf[0]} << endl;
					
					for(i = 0; i < sizeof(read_buf); i++){//find the head point
						//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
						if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
						 	break;
						} 
					}
					i+=2;	
					//printf("7point(%d\n", i);
					//---------------------------------------------------
					memset(&g, '\0', sizeof(g));
					memcpy(&g, &read_buf[i], sizeof(g));
					//for(int l = 0; l < sizeof(g); l++) cout << "|gdata(" <<  std::bitset<8>{g[l]} << endl;//data check
					
					memcpy(&bset12, &bset1, 1);
					memcpy(&bset22, &bset2, 1);
					memcpy(&bset32, &bset3, 1);
					memcpy(&bset42, &bset4, 1);
					memcpy(&bset52, &bset5, 1);
					
					memset(&bset1, '\0', sizeof(bset1));//fill in the array
					memset(&bset2, '\0', sizeof(bset2));
					memset(&bset3, '\0', sizeof(bset3));
					memset(&bset4, '\0', sizeof(bset4));
					memset(&bset5, '\0', sizeof(bset5));
					
					memcpy(&g, &read_buf[i], sizeof(g));
					memcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << 8;memcpy(&bset1, &g[1], 1);//read buff
					memcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << 8;memcpy(&bset2, &g[3], 1);
					memcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << 8;memcpy(&bset3, &g[5], 1);
					memcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << 8;memcpy(&bset4, &g[7], 1);
					memcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << 8;memcpy(&bset5, &g[9], 1);
				
				
				
	 				if(min1[0] == 0 && bset1[0] != 0 && (bset1[0]-bset12[0]) >= 0)memcpy(&min1,&bset1,1);
					if(min2[0] == 0 && bset2[0] != 0 && (bset2[0]-bset22[0]) >= 0)memcpy(&min2,&bset2,1);
					if(min3[0] == 0 && bset3[0] != 0 && (bset3[0]-bset32[0]) >= 0)memcpy(&min3,&bset3,1);
					if(min4[0] == 0 && bset4[0] != 0 && (bset4[0]-bset42[0]) >= 0)memcpy(&min4,&bset4,1);
					if(min5[0] == 0 && bset5[0] != 0 && (bset5[0]-bset52[0]) >= 0)memcpy(&min5,&bset5,1);

					
					if((min1[0] != 0) && (min2[0] != 0) && (min3[0] != 0) && (min4[0] != 0) && (min5[0] != 0)){
					oneces++;
					
					break;
					}
				}
			}
			cout << "b1data(" << std::dec << (bset1[0]) << endl;
			//cout << "b2data(" << std::bitset<16>{bset2[0]} << endl;
			cout << "b2data(" << std::dec << (bset2[0]) << endl;
	   		//cout << "b3data(" << std::bitset<16>{bset3[0]} << endl;
			cout << "b3data(" << std::dec << (bset3[0]) << endl;
			//cout << "b4data(" << std::bitset<16>{bset4[0]} << endl;
			cout << "b4data(" << std::dec << (bset4[0]) << endl;
			//cout << "b5data(" << std::bitset<16>{bset5[0]} << endl;
			cout << "b5data(" << std::dec << (bset5[0]) << endl;		
			int border = 130;
			if((bset1[0] > border)&&(bset2[0] > border)&&(bset3[0] > border)&&(bset4[0] > border)&&(bset5[0] > border)){
				image = Scalar(255, 255, 255);
				uint16_t scor = 710;
			
				float mh1 = (bset1[0] - min1[0]);float mh12 = (scor - min1[0]);
				float mh2 = (bset2[0] - min2[0]);float mh22 = (scor - min2[0]);
				float mh3 = (bset3[0] - min3[0]);float mh32 = (scor - min3[0]);
				float mh4 = (bset4[0] - min4[0]);float mh42 = (scor - min4[0]);
				float mh5 = (bset5[0] - min5[0]);float mh52 = (scor - min5[0]);
				
				float s1 = mh1/mh12;
				float s2 = mh2/mh22; 
				float s3 = mh3/mh32;
				float s4 = mh4/mh42;
				float s5 = mh5/mh52;
				//printf("%f\n", s1);
				//std::cout << s5 << "," << s2 << "," << s3 << "," << s4 <<std::endl;
				circle(image, Point(250,250), (35+(65*s1)), cv::Scalar(255, 0,0),2);//center
				circle(image, Point(250,125), (35+(65*s2)), cv::Scalar(255, 0,0),2);//12
				circle(image, Point(250,375), (35+(65*s3)), cv::Scalar(255, 0,0),2);//6
				circle(image, Point(125,250), (35+(65*s4)), cv::Scalar(255, 0,0),2);//9
		  		circle(image, Point(375,250), (35+(65*s5)), cv::Scalar(255, 0,0),2);//3
				int x = (((s1*250)+(s2*250)+(s3*250)+(s4*25)+(s5*475))/(s1+s2+s3+s4+s5));
				int y = (((s1*250)+(s2*25)+(s3*475)+(s4*250)+(s5*250))/(s1+s2+s3+s4+s5));
				
				circle(image,Point(x,y), 10, cv::Scalar(0,0,255),-1);
			//}
			imshow("img",image);
			key = waitKey(10);
			//printf("\n");
			}
			
		}
  	}
  	
  	return 0;
  	
}
//-------------------------------

/*
		if(key == 99){	//Clear screen with c key
			watch = Scalar(255, 255, 255);
			pos = Point(-10,-10);
		}
		
		if(key == 97){}	//Simulate with a key
*/	


/*
void 2Bval(uint16_t data){
	memcpy(&data[0], &g[va], 1);
	data[0] = data[0] << 8;
	memcpy(&data, &g[va+1], 1);
	cout << "cpdata(" << std::bitset<16>{data[0]} << endl;
	va+=2;
	if(va => 10)va = 0;
}
*/
//------------------end-------------------------	
	
/*
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <unistd.h> 
#include <sys/ioctl.h>
#include <bitset>

using namespace cv;
using namespace std;
int r = 5;
int p = 0;

Point pos(-10,-10);//初期位置
Mat	koba(500, 500, CV_8UC3);//y,x

void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
	bool *isClick = static_cast<bool *>(userdata);
    if (event == EVENT_LBUTTONDOWN) {
		*isClick = true;
		cout << "(" << x << ", " << y << ")" << endl;
		pos = Point(x,y);
    }
}


int main(int argc, char *argv[])
{
	
	bool isClick = false;
	

	koba = Scalar(255, 255, 255);

	while(1)
	{
		int key;
		int nv = 4;//number of vertices
		int s = 1;diameter
		int r = 5;radius
		int p =0;
		int c =1;
		
		setMouseCallback("img", mouse_callback, &isClick);
		Mat	img(400, 400, CV_8UC3);
		imshow("img",koba);
		//if(isClick == true) {isClick = false;}
		
		
		key = waitKey(50);
		if(key == 99){	//Clear screen with c key
			koba =Scalar(255, 255, 255);
			pos= Point(-10,-10);
		}
		
		if(key == 97){	//Simulate with a key
			while(1){
			r++;
				s=(r*4);	
				
				while(1){		
				pos= Point((cos(CV_PI * (2.0*p/n - 0.5)) * 140 + 250),\
				(sin(CV_PI * (2.0*p/n - 0.5)) * 140 + 250));
				p++;
				circle(koba, pos, s, cv::Scalar(255, 0,0),1);
				imshow("img",koba);
				cout << "("<< p <<";"<<s<<")"<<endl;
				if(p==4){break;}
				}
				
				
				circle(koba, Point(250,250), s, cv::Scalar(255, 0,0),1);
				imshow("img",koba);
				if(p == n){koba =Scalar(255, 255, 255);}
				if(p == n){p=0;}
				if(s > 80){r=5;}
				//if(i == 4){i = 0;}
				//cout << "("<< p <<";"<<s<<")"<<endl;
				k = waitKey(50);
				
			}
*/
//--------------------------verification
/*
		cout << "10othx(" << std::hex << bset1[0] << endl;		
		printf("11otoc(%d\n\n", bset1[0]);
		cout << "8gdata(" << std::bitset<16>{bset1[0]} << endl;
		bset1[0] = bset1[0] << 8;
		cout << "8gdata(" << std::bitset<16>{bset1[0]} << endl;
		bset1[0] = bset1[0] | bset12[0] ;			
		cout << "8gdata(" << std::bitset<16>{bset1[0]} << endl;
		*/	
//------------------------------------------------
		//cout << "8gdata(" << std::bitset<8>{g[0]} << endl;
		//cout << "9Mdata(" << std::hex << (read_buf[i]-1+1) << endl;//bag?
		//cout << "b1data(" << std::bitset<16>{bset1[0]} << endl;
		//cout << "b1data(" << std::dec << (bset1[0]) << endl;
		//cout << "b2data(" << std::bitset<16>{bset2[0]} << endl;
		//cout << "b2data(" << std::dec << (bset2[0]) << endl;
		//cout << "b3data(" << std::bitset<16>{bset3[0]} << endl;
		//cout << "b3data(" << std::dec << (bset3[0]) << endl;
		//cout << "b4data(" << std::bitset<16>{bset4[0]} << endl;
		//cout << "b4data(" << std::dec << (bset4[0]) << endl;
		//cout << "b5data(" << std::bitset<16>{bset5[0]} << endl;
		//cout << "b5data(" << std::dec << (bset5[0]) << endl;
		
//--------------------------------------

	/*--------------
	while(1){
		ioctl(serial_port, FIONREAD, &bytesWaiting);
		
		if (bytesWaiting > 30){
	   		uint8_t read_buf [bytesWaiting+1];
      		memset(&read_buf, '\0', sizeof(read_buf));
       		int n = read(serial_port, &read_buf, sizeof(read_buf));
       		
        	for(i = 0; i < sizeof(read_buf); i++){
				//cout << "|Mdata(" << std::hex << read_buf[i] << endl;
				cout << "|r_bby(" << std::bitset<16>{read_buf[i]} << endl;
			} 
		printf(";\n");
		}
	}
	
	uint8_t g[10];
	uint16_t bset1 [1];
	uint16_t bset2 [1];
	uint16_t bset3 [1];
	uint16_t bset4 [1];
	uint16_t bset5 [1];
	uint16_t bset12 [1];
	uint16_t bset22 [1];
	uint16_t bset32 [1];
	uint16_t bset42 [1];
	uint16_t bset52 [1];
	uint16_t min1 [1];
	uint16_t min2 [1];
	uint16_t min3 [1];
	uint16_t min4 [1];
	uint16_t min5 [1];
	int i;
	int va = 0;
	int oneces = 0;	
		
		
	uint8_t bytesWaiting;
	Point pos(-10,-10);
	
    while(1){
 		ioctl(serial_port, FIONREAD, &bytesWaiting);	//read data from serial

		if (bytesWaiting > 25){
			uint8_t read_buf [bytesWaiting+1];
		  	memset(&read_buf, '\0', sizeof(read_buf));
		  	read(serial_port, &read_buf, sizeof(read_buf));
							
			//printf("2bsize(%d\n", bytesWaiting);		
			//printf("3r_bsz(%ld\n", sizeof(read_buf));
			//cout << "6r_bBy(" << std::bitset<16>{read_buf[0]} << endl;
			
			for(i = 0; i < sizeof(read_buf); i++){//find the head point
				//cout << "|check(" <<  std::bitset<8>{read_buf[i]} << endl;//data check
				if((read_buf[i] == 0xff)&&(read_buf[i+1] == 0xff)&&(read_buf[i+2] != 0xff)&&(read_buf[i+12] == 0xff)&&(read_buf[i+13] == 0xff)){
				 	break;
				 	} 
			 	}
			i+=2;	
			//printf("7point(%d\n", i);
			//---------------------------------------------------
			memset(&g, '\0', sizeof(g));
			memcpy(&g, &read_buf[i], sizeof(g));
			//for(int l = 0; l < sizeof(g); l++) cout << "|gdata(" <<  std::bitset<8>{g[l]} << endl;//data check
			
			memset(&bset1, '\0', sizeof(bset1));//fill in the array
			memset(&bset2, '\0', sizeof(bset2));
			memset(&bset3, '\0', sizeof(bset3));
			memset(&bset4, '\0', sizeof(bset4));
			memset(&bset5, '\0', sizeof(bset5));
			
			memcpy(&g, &read_buf[i], sizeof(g));
			memcpy(&bset1, &g[0], 1);bset1[0] = bset1[0] << 8;memcpy(&bset1, &g[1], 1);//read buff
			memcpy(&bset2, &g[2], 1);bset2[0] = bset2[0] << 8;memcpy(&bset2, &g[3], 1);
			memcpy(&bset3, &g[4], 1);bset3[0] = bset3[0] << 8;memcpy(&bset3, &g[5], 1);
			memcpy(&bset4, &g[6], 1);bset4[0] = bset4[0] << 8;memcpy(&bset4, &g[7], 1);
			memcpy(&bset5, &g[8], 1);bset5[0] = bset5[0] << 8;memcpy(&bset5, &g[9], 1);
			
			
			
			int border = 155;
			if((bset1[0] > border)&&(bset2[0] > border)&&(bset3[0] > border)&&(bset4[0] > border)&&(bset5[0] > border)){
				image = Scalar(255, 255, 255);
				
				circle(image, Point(250,250), (bset1[0]/15*2), cv::Scalar(255, 0,0),2);//center
				circle(image, Point(250,125), (bset2[0]/15*2), cv::Scalar(255, 0,0),2);//12
				circle(image, Point(250,375), (bset3[0]/15*2), cv::Scalar(255, 0,0),2);//6
				circle(image, Point(125,250), (bset4[0]/15*2), cv::Scalar(255, 0,0),2);//9
				circle(image, Point(375,250), (bset5[0]/15*2), cv::Scalar(255, 0,0),2);//3
				int x = (((bset1[0]*250)+(bset2[0]*250)+(bset3[0]*250)+(bset4[0]*25)+(bset5[0]*475))/(bset1[0]+bset2[0]+bset3[0]+bset4[0]+bset5[0]));
				int y = (((bset1[0]*250)+(bset2[0]*25)+(bset3[0]*475)+(bset4[0]*250)+(bset5[0]*250))/(bset1[0]+bset2[0]+bset3[0]+bset4[0]+bset5[0]));
				std::cout << x << "," << y << std::endl;
				circle(image,Point(x,y), 3, cv::Scalar(0,0,255),-1);
			}
			imshow("img",image);
			key = waitKey(10);
			//printf("\n");
		}
  	}
  	//return 0;
}
//-------------------------------

	*/
	
