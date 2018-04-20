#include <stdio.h>
//#include "allocore/al_Allocore.hpp"

#include "al/core.hpp"
//#include "allocore/graphics/al_Font.hpp"
#include "al/util/al_Font.hpp"
//#include "allocore/ui/al_Parameter.hpp"
#include "al/util/ui/al_Parameter.hpp"
//#include "alloutil/al_AlloSphereSpeakerLayout.hpp"
#include "al/util/al_AlloSphereSpeakerLayout.hpp"
#include "Gamma/SamplePlayer.h"
//#include "alloaudio/al_SoundfileBuffered.hpp"
//#include "al/core/sound/
using namespace al;

#define BLOCK_SIZE (1024)

//#define SKIP_FOR_DEBUG

int sampleRate = 44100;
int outputChannels = 0;
static std::atomic<float> *mPeaks;


ParameterServer paramServer("127.0.0.1",8080);
Parameter srcAzimuth("srcAzimuth","",0.0);
Parameter srcElevation("srcElevation","",-0.1);
Parameter sourceSound("sourceSound","",0);

float srcRadius = 6.0;

gam::SamplePlayer<> player;


std::string path = "/Users/sphere/code/nathan/Gamma/sounds/count.wav";
//SoundFileBuffered soundFileBuffered(path, true);

float *sfBuffer;


//std::string path = "/Users/primary1/Documents/code/AlloSystemTest/Gamma/sounds/count.wav";

class SpeakerLBAP{
public:
  int channel;

  //Angle from forward to right vector
  float azimuth;
  float elevation;
  Vec3f position;

  SpeakerLBAP *leftSpeaker;
  SpeakerLBAP *rightSpeaker;


  SpeakerLBAP(int channel=0,float angle=0.0,float elevation=0.0, float radius=1.0, bool isLeftHanded=true): elevation(elevation), channel(channel) {

      azimuth = toRad(angle);

      if(azimuth < 0.0){
          azimuth += M_2PI;
      }

      if(!isLeftHanded){
          azimuth = M_2PI - azimuth;
      }

      double cosel = cos(toRad(elevation));
      double x = sin(azimuth) * cosel * radius;
      double y = cos(azimuth) * cosel * radius;
      double z = sin(toRad(elevation)) * radius;

      position = Vec3d(x,y,z);
      this->elevation = toRad(elevation);
  }

  static double toRad(double d){ return d*M_PI/180.; }
};

bool speakerSort(SpeakerLBAP const &first, SpeakerLBAP const &second){
    return first.azimuth < second.azimuth;
}

class SpeakerLayer {
public:
    float elevation;

    std::vector<SpeakerLBAP> speakers;

    SpeakerLayer(float elevation): elevation(toRad(elevation)){

    }

//    bool SpeakerLayer::speakerSort(SpeakerLBAP const &first, SpeakerLBAP const &second){
//        return first.azimuth < second.azimuth;
//    }

    int initLayer(){
        std::sort(speakers.begin(),speakers.end(),&speakerSort);
        for(int i = 0; i < speakers.size(); i++){
            int rightSpeakerPos = (i+1) % speakers.size();
            int leftSpeakerPos = i-1;
            if(leftSpeakerPos < 0){
                leftSpeakerPos += speakers.size();
             }
            speakers[i].rightSpeaker = &speakers[rightSpeakerPos];
            speakers[i].leftSpeaker = &speakers[leftSpeakerPos];
        }
        return 1;
    }

    static double toRad(double d){ return d*M_PI/180.; }

};



std::vector<SpeakerLayer> speakerLayers;

Vec3d convertCoords(Vec3d v){
    Vec3d returnVector;
    returnVector.x = v.x;
    returnVector.y = v.z;
    returnVector.z = -v.y;
    return returnVector;
}

void spherToCartCoords(Vec3f &vec){

    float z = vec[2]*sin(vec[1]);
    float a = vec[2]*cos(vec[1]);
    float x = a*cos(vec[0]);
    float y = a*sin(vec[0]);

    vec[0] = x;
    vec[1] = y;
    vec[2] = z;


    //printf("x: %f y: %f z: %f \n", x,y,z);
}

void cartToSphericalCoords(Vec3f &vec){

    float radius = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
    float elevation  = asin(vec.z/radius);
    float azimuth = atan2(vec.y,vec.x);

    vec[0] = azimuth;
    vec[1] = elevation;
    vec[2] = radius;

   // spherToCartCoords(vec);

    //printf("azimuth: %f elev: %f  radius: %f \n", azimuth,elevation,radius);

}




class MyApp : public App
{
#ifndef SKIP_FOR_DEBUG
    Font mFont;
    Mesh mText;
#endif


    Mesh mSpeakerMesh;

    Light mLight;

public:
//    MyApp() /*: mFont("allocore/share/fonts/VeraMoIt.ttf", 20)*/ {
     MyApp() /*: mFont("allocore/share/fonts/VeraMoIt.ttf", 20)*/ {

#ifndef SKIP_FOR_DEBUG
        mFont.load("allocore/share/fonts/VeraMoIt.ttf", 20);
        addSurface(mText,0,4);
#endif
        mLight.pos(0,6,0);

//        addSurface(mText,0,4);
        addSphere(mSpeakerMesh);

        nav().pos(0, 1.0, 20.0);
        initWindow();

        player.load("/Users/primary1/Documents/code/AlloSystemTest/Gamma/sounds/water3.wav");
    }


    void onDraw(Graphics& g){
        g.blendAdd();

        //Draw the source
        g.pushMatrix();

        double cosel = cos(srcElevation.get());
        double x = sin(srcAzimuth.get()) * cosel * srcRadius;
        double y = cos(srcAzimuth.get()) * cosel * srcRadius;
        double z = sin(srcElevation.get()) * srcRadius;
        g.translate(convertCoords(Vec3d(x,y,z)));
        g.scale(0.1);
        g.color(1.0,0.0,0.0);
        g.draw(mSpeakerMesh);
        g.popMatrix();

        //Draw the speakers
        for(int i = 0; i < speakerLayers.size(); ++i){

            std::vector<SpeakerLBAP> aSpeakerLayer = speakerLayers[i].speakers;
            //int peakIndex = 0;
            for(int j = 0; j < aSpeakerLayer.size(); j++){


                g.pushMatrix();
                if(i==0){
                    g.color(1.0,0.0,0.0);
                }else{
                    g.color(1.0,1.0,1.0);
                }
                g.translate(convertCoords(aSpeakerLayer[j].position));

                //Draw Text Label
                g.pushMatrix();
                g.scale(0.01);
                g.translate(0.0,10.0,0.0);

#ifndef SKIP_FOR_DEBUG
                mFont.write(mText,std::to_string(aSpeakerLayer[j].channel));
                mFont.texture().bind();
                g.draw(mText);
                mFont.texture().unbind();
#endif

                g.popMatrix();

                //g.scale(0.05);
               // float peak = mPeaks[peakIndex].load();
                 float peak = mPeaks[aSpeakerLayer[j].channel].load();
                //peakIndex++;
                g.scale(0.02 + 0.04 * peak * 30);
                g.draw(mSpeakerMesh);
                g.popMatrix();

        }
    }
    }
};

bool isAngleBetween(float target, float angle1, float angle2){
    float cAngle = fmodf(fmodf(angle2-angle1,M_2PI)+ M_2PI,M_2PI);

    if(cAngle >= M_2PI){
        std::swap(angle1,angle2);
    }

    if(angle1 <= angle2){
        return target >= angle1 && target <= angle2;
    }else{
        return target >= angle1 || target <= angle2;
    }
}

//returns difference < PI
float diffOfAngles(float angle1, float angle2){
    return M_PI-fabsf(fabsf(angle1-angle2)-M_PI);
}

void findSpeakerPair(float sourceAzimuth, SpeakerLayer *layer, SpeakerLBAP*& leftSpeaker, SpeakerLBAP*& rightSpeaker){

    for (int i = 0; i < layer->speakers.size(); i++){
        if(isAngleBetween(sourceAzimuth,layer->speakers[i].azimuth,layer->speakers[i].rightSpeaker->azimuth)){
            leftSpeaker = &layer->speakers[i];
            rightSpeaker = layer->speakers[i].rightSpeaker;
            break;
        }
    }
}

std::vector<float> calculateGains(){

    std::vector<float> out(outputChannels,0.0);

    Vec3f cords(srcAzimuth.get(),srcElevation.get(),srcRadius);


    //Find another way to do this
    spherToCartCoords(cords);
    cartToSphericalCoords(cords);

    float srcAzi = cords[0];
    float srcElev = cords[1];


//    float srcElev = srcElevation.get();
//    float srcAzi = srcAzimuth.get();

    while(srcAzi < 0){
        srcAzi += M_2PI;
    }

    if(srcElev < speakerLayers[0].elevation){
        //Below lowest layer

        SpeakerLBAP *left, *right;
        findSpeakerPair(srcAzi,&speakerLayers[0],left,right);
        float dist = diffOfAngles(srcAzi,left->azimuth) / diffOfAngles(right->azimuth,left->azimuth);

        out[left->channel] = cos(dist*M_PI_2);
        out[right->channel] = sin(dist*M_PI_2);

        return out;
    }else if(srcElev > speakerLayers[speakerLayers.size()-1].elevation){
        //Above highest layer

        SpeakerLBAP *left, *right;
        findSpeakerPair(srcAzi,&speakerLayers[speakerLayers.size()-1],left,right);
        float dist = diffOfAngles(srcAzi,left->azimuth) / diffOfAngles(right->azimuth,left->azimuth);

        out[left->channel] = cos(dist*M_PI_2);
        out[right->channel] = sin(dist*M_PI_2);

        return out;
    }

    SpeakerLayer *bottom;
    SpeakerLayer *upper;

    SpeakerLBAP *bottomLeft, *bottomRight, *upperLeft, *upperRight;
 //       SpeakerLBAP bottomLeft, bottomRight, upperLeft, upperRight;

    for(int i = 0; i < speakerLayers.size()-1; i++){

        if(isAngleBetween(srcElev,speakerLayers[i].elevation,speakerLayers[i+1].elevation)){

            bottom = &speakerLayers[i];
            upper = &speakerLayers[i+1];
            break;
        }
    }

    findSpeakerPair(srcAzi,bottom,bottomLeft,bottomRight);

   // printf("bottomLeft azi %f \n", bottomLeft->azimuth);

//    for (int i = 0; i < bottom->speakers.size(); i++){
////        if(bottom->speakers[i].azimuth <= srcAzi && bottom->speakers[i].rightSpeaker->azimuth > srcAzi){
//        if(isAngleBetween(srcAzi,bottom->speakers[i].azimuth,bottom->speakers[i].rightSpeaker->azimuth)){
//            bottomLeft = &bottom->speakers[i];
//            bottomRight = bottom->speakers[i].rightSpeaker;
//            break;
//        }
//    }

    findSpeakerPair(srcAzi,upper,upperLeft,upperRight);

//    for (int i = 0; i < upper->speakers.size(); i++){
////        if(upper->speakers[i].azimuth <= srcAzi && upper->speakers[i].rightSpeaker->azimuth > srcAzi){
//        if(isAngleBetween(srcAzi, upper->speakers[i].azimuth,upper->speakers[i].rightSpeaker->azimuth)){
//            upperLeft = &upper->speakers[i];
//            upperRight = upper->speakers[i].rightSpeaker;
//            break;
//        }
//    }

   float belowDist = (srcElev - bottom->elevation) / (upper->elevation - bottom->elevation);

   float aboveAmp = cos(belowDist*M_PI_2);
   float belowAmp = sin(belowDist*M_PI_2);

   float blDist = diffOfAngles(srcAzi,bottomLeft->azimuth) / diffOfAngles(bottomRight->azimuth,bottomLeft->azimuth);
   float alDist = diffOfAngles(srcAzi,upperLeft->azimuth) / diffOfAngles(upperRight->azimuth,upperLeft->azimuth);

   out[bottomLeft->channel] = cos(blDist*M_PI_2)*cos(belowAmp*M_PI_2);
   out[bottomRight->channel] = sin(blDist*M_PI_2)*cos(belowAmp*M_PI_2);

   out[upperLeft->channel] = cos(alDist*M_PI_2)*cos(aboveAmp*M_PI_2);
   out[upperRight->channel] = sin(alDist*M_PI_2)*cos(aboveAmp*M_PI_2);

   return out;
}

static void audioCB(AudioIOData& io){

    static unsigned int t = 0;
    std::vector<float> gains = calculateGains();

    float srcSound = sourceSound.get();

    //soundFileBuffered.read(sfBuffer,BLOCK_SIZE);

    while (io()) {

        if(srcSound == 0){
       double sec = (t / io.fps());

       for(int i = 0; i < outputChannels; i++){
           io.out(i) = 0.5 * sin(sec*440*M_2PI) * gains[i] ;
       }

       t++;
        } else if(srcSound == 1){



            for(int i = 0; i < outputChannels; i++){
//                io.out(i) = io.in(0) * gains[i] ;
                io.out(i) = sfBuffer[io.frame()] * gains[i] ;
            }

//             float sample = player.read(0) * 0.2;
//             player.advance();
//             player.loop();
//             for(int i = 0; i < outputChannels; i++){
// //                io.out(i) = io.in(0) * gains[i] ;
//                 io.out(i) = sample * gains[i] ;
//             }
        }
    }

    for (int i = 0; i < outputChannels; i++) {
        mPeaks[i].store(0);
    }
    for (int channel = 0; channel < outputChannels; channel++) {
        float rms = 0;
        for (int i = 0; i < io.framesPerBuffer(); i++) {
            //int deviceChannel = channel;
            if(channel < io.channelsOut()) {
                float sample = io.out(channel, i);
                rms += sample * sample;
            }
        }
        rms = sqrt(rms/io.framesPerBuffer());
        mPeaks[channel].store(rms);
    }

}

int main(int argc, char *argv[] ) {

//    Vec3f coords(-3.0,3.0,-3.0);

//    cartToSphericalCoords(coords);

    // std::string path = "/Users/sphere/code/nathan/Gamma/sounds/water4.wav";

    // SoundFileBuffered Buffered(path, true);

    sfBuffer = (float*) calloc (BLOCK_SIZE,sizeof(float));

    Vec3f sphereCoords(2.356194,2.44,5.19615);

    spherToCartCoords(sphereCoords);
    cartToSphericalCoords(sphereCoords);

    paramServer.registerParameter(srcAzimuth);
    paramServer.registerParameter(srcElevation);
    paramServer.registerParameter(sourceSound);

    float radius = 5.0;

//    SpeakerLayer sl(-32.0);
//    sl.speakers.push_back(SpeakerLBAP(0,-45.0,-32.0,radius));
//    sl.speakers.push_back(SpeakerLBAP(1,45.0,-32.0,radius));
//    sl.speakers.push_back(SpeakerLBAP(2,135.0,-32.0,radius));
//    sl.speakers.push_back(SpeakerLBAP(3,225.0,-32.0,radius));
//    sl.initLayer();
//    speakerLayers.push_back(sl);

//    SpeakerLayer sl1(0.0);
//    sl1.speakers.push_back(SpeakerLBAP(4,-45.0,0.0,radius));
//    sl1.speakers.push_back(SpeakerLBAP(5,45.0,0.0,radius));
//    sl1.speakers.push_back(SpeakerLBAP(6,135.0,0.0,radius));
//    sl1.speakers.push_back(SpeakerLBAP(7,225.0,0.0,radius));
//    sl1.initLayer();
//    speakerLayers.push_back(sl1);

    SpeakerLayer slBottom(-32.5);
    SpeakerLayer slMiddle(0.0);
    SpeakerLayer slTop(41.0);

    AlloSphereSpeakerLayout alloLayout;
    Speakers alloSpeakers = alloLayout.speakers();

    //in radians
    float elevationTolerance = 0.01745;

    for(int i = 0; i < alloSpeakers.size();i++){
        Speaker sp = alloSpeakers[i];

        float elevationRad = sp.elevation * M_PI/180.0;

        if(elevationRad > slBottom.elevation-elevationTolerance && elevationRad < slBottom.elevation + elevationTolerance){
            slBottom.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,false));
        }else if(elevationRad > slMiddle.elevation-elevationTolerance && elevationRad < slMiddle.elevation + elevationTolerance){
            slMiddle.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,false));
        }else if(elevationRad > slTop.elevation-elevationTolerance && elevationRad < slTop.elevation + elevationTolerance){
            slTop.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,false));
        }
    }

    slBottom.initLayer();
    slMiddle.initLayer();
    slTop.initLayer();

    speakerLayers.push_back(slBottom);
    speakerLayers.push_back(slMiddle);
    speakerLayers.push_back(slTop);

    printf("layer1 %f \n",speakerLayers[1].elevation);

    int numSpeakers = 0;
    int highestChannel = 0;
    for(int i = 0 ; i < speakerLayers.size(); i++){

        std::vector<SpeakerLBAP> aSpeakerLayer = speakerLayers[i].speakers;
        numSpeakers += aSpeakerLayer.size();

        for(int j = 0; j < aSpeakerLayer.size(); j++){
            if(aSpeakerLayer[j].channel > highestChannel){
                highestChannel = aSpeakerLayer[j].channel;
            }
        }
    }

    outputChannels = highestChannel + 1;
    printf("highestChannel %i",highestChannel);
    mPeaks = new std::atomic<float>[outputChannels];

    //zero mPeaks
    for (int i = 0; i < outputChannels; i++) {
        mPeaks[i].store(0);
    }

    AudioIO audioIO(BLOCK_SIZE, sampleRate, audioCB, 0, outputChannels, 1);
    audioIO.device(AudioDevice("ECHO X5"));
    audioIO.start();

    audioIO.print();

    MyApp().start();

    getchar();
    return 0;
}
