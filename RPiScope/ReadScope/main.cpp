/*
	Developed by Daniel Pelikan 2013,2014
	http://digibird1.wordpress.com/
*/
#include <iostream>
#include <cmath>
#include <fstream>
#include <bitset>
#include <cstdlib>
#include <time.h>

typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;

#define SAMPLE_SIZE 1000000

//---------------------------------------------------------------------------------------------------------
int main(){

    //Read the RPi
    uint16_t Buffer[SAMPLE_SIZE];
    uint8_t *ScopeBufferStart;
    uint8_t *ScopeBufferStop;
    uint8_t *buf_p;

    buf_p=(uint8_t*)Buffer;
    ScopeBufferStart=(uint8_t*)&Buffer;
    ScopeBufferStop=ScopeBufferStart+sizeof(Buffer);

    //std::cout << Buffer << "\t"<<ScopeBufferStart << "\t"<< ScopeBufferStop << std::endl;
    
    std::string line;
    std::ifstream devfile ("/dev/linet");
    if (devfile.is_open())
    {
      while ( std::getline (devfile,line) )
      {
        for(uint32_t i=0;i<line.size();i++){
          if(buf_p>ScopeBufferStop) std::cerr<<"buf_p out of range!"<<std::endl;
          *(buf_p)=line[i];
          buf_p++;
        }
      }
      devfile.close();
    }
    else std::cerr << "Unable to read file"<<std::endl;
//---------------------------------------------------------------------------------------------------------
    //copy datas to another file
    //fprintf 往文件中写格式化数据

    time_t nowtime = time(NULL);
    struct tm *p = gmtime(&nowtime); 
    
    char filename[128] = {0};

    sprintf(filename,"./data/%d%02d%d_%d-%02d-%02d.txt",1900+p->tm_year,1+p->tm_mon,p->tm_mday,8+p->tm_hour,p->tm_min,p->tm_sec);
    printf("file crated at %s", filename);
    //file saved under data/
    std::ofstream myfile (filename);
    if (myfile.is_open())
    {
      for(int i=0;i<SAMPLE_SIZE;i++)
      {
        short tmp = Buffer[i] & 0xfff;
        signed short valueADC1 = ((tmp > 2047)? (2048-tmp) : tmp);
        uint16_t channel1 = Buffer[i]>>12;
        myfile << channel1 << "\t" <<valueADC1<<"\t"<< std::bitset<16>(Buffer[i]) << std::endl;
      }
      myfile.close();
    }
    else std::cerr << "Unable to write file"<<std::endl;

    return 0;
}
