#include <Arduino.h>
#include <vector>
#include <deque>


#include "Adafruit_TinyUSB.h"
HardwareSerial SerialA(USART1); 

#define PREAMBLE0 '<'
#define PREAMBLE1 '$'
#define PREAMBLE2 'M'
#define PREAMBLE3 'U'

// ---------- ARG ----------
enum MRType { MR_INT=78, MR_BYTES=65, MR_STR=83, MR_BOOL=66 };

struct MRArg {
    MRType type;
    std::vector<uint8_t> data;
};

struct MRPacket {
    String cmd;
    std::vector<MRArg> args;
};

// =====================================================

class MicroRemote {

public:
    Stream &io;

    MicroRemote(Stream &s):io(s){}

    enum State {WAIT_LEN,WAIT_PREAMBLE,WAIT_PAYLOAD};
    State state=WAIT_LEN;

    uint8_t expected_len=0;
    uint8_t pre_idx=0;
    std::vector<uint8_t> buf;

    std::deque<MRPacket> rxq;
    std::deque<std::vector<uint8_t>> txq;

    // ---------- RX ----------
    void pollRX(){

        while(io.available()>0){

            uint8_t b=io.read();
            Serial.print("b=");
            Serial.print(b,HEX);
            Serial.print(" ");
            switch(state){

            case WAIT_LEN:
                expected_len=b;
                buf.clear();
                pre_idx=0;
                state=WAIT_PREAMBLE;
                break;

            case WAIT_PREAMBLE:
                if(b!=PREAMBLE0+pre_idx*0){ // trick to silence warnings
                    if((pre_idx==0&&b!=PREAMBLE0)||
                       (pre_idx==1&&b!=PREAMBLE1)||
                       (pre_idx==2&&b!=PREAMBLE2)||
                       (pre_idx==3&&b!=PREAMBLE3)){
                        state=WAIT_LEN;
                        break;
                    }
                }
                pre_idx++;
                if(pre_idx==4) state=WAIT_PAYLOAD;
                break;

            case WAIT_PAYLOAD:
                buf.push_back(b);
                if(buf.size()==expected_len-4){
                    decode(buf);
                    state=WAIT_LEN;
                }
                break;
            }
        }
    }

    // ---------- DECODE ----------
    void decode(const std::vector<uint8_t>& in){

        MRPacket p;

        int cmd_len=in[0];
        p.cmd=String((char*)&in[1]).substring(0,cmd_len);

        int i=1+cmd_len;

        while(i<in.size()){
            MRArg a;
            a.type=(MRType)in[i++];
            int l=in[i++];
            a.data.insert(a.data.end(),in.begin()+i,in.begin()+i+l);
            i+=l;
            p.args.push_back(a);
        }

        rxq.push_back(p);
    }

    // ---------- ENCODE ----------
    std::vector<uint8_t> encode(const char *cmd,const std::vector<MRArg>& args){

        std::vector<uint8_t> out;

        uint8_t l=strlen(cmd);
        out.push_back(l);
        out.insert(out.end(),cmd,cmd+l);

        for(auto &a:args){
            out.push_back(a.type);
            out.push_back(a.data.size());
            out.insert(out.end(),a.data.begin(),a.data.end());
        }
        return out;
    }

    void queue(const char*cmd,const std::vector<MRArg>&args){

        auto p=encode(cmd,args);

        std::vector<uint8_t> f;
        f.push_back(p.size()+4);
        f.insert(f.end(),{'<','$','M','U'});
        f.insert(f.end(),p.begin(),p.end());

        txq.push_back(f);
    }

    void pollTX(){
        if(txq.empty()) return;

        auto &f=txq.front();
        io.write(f.data(),f.size());
        txq.pop_front();
    }

    template<typename F>
    void process(F handler){

        pollRX();

        if(!rxq.empty()){
            auto p=rxq.front();
            rxq.pop_front();

            auto resp=handler(p.cmd,p.args);
            String ack=p.cmd+"_ack";
            queue(ack.c_str(),resp);
        }

        pollTX();
    }
};


MRArg MR_int(int v){
    MRArg a; a.type=MR_INT;
    char buf[16]; itoa(v,buf,10);
    a.data.assign(buf,buf+strlen(buf));
    return a;
}

int to_int(const MRArg&a){ return atoi((char*)a.data.data()); }
bool to_bool(const MRArg&a){ return a.data[0]!=0; }

std::vector<MRArg> handler(String cmd,const std::vector<MRArg>&args){

    if(cmd=="ping")
        Serial.println("ping");
        return {MR_int(millis())};

    if(cmd=="add")
        Serial.println("add");
        return {MR_int(to_int(args[0])+to_int(args[1]))};

    if(cmd=="led"){
        Serial.println("led");
        digitalWrite(PC13,to_bool(args[0]));
        return {MR_int(1)};
    }

    return {MR_int(-1)};
}

// MicroRemote usb(SerialUSB);
MicroRemote uart(SerialA);

void setup(){

     if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
 
    pinMode(PA10, INPUT);
  
    SerialA.begin(115200);
    SerialA.println("dit is vanaf SerialA");
    delay(1000);
    Serial.println("hello");
    //while(!SerialUSB); // wait PC
}

void loop(){
    //Serial.println("listening");
    //delay(500);
   
     //uart.process(handler);
     while (SerialA.available()>0) {
        uint8_t a = SerialA.read();
        Serial.print(a,HEX);
        Serial.print(" ");

     }
}
