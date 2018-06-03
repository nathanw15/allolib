#include <stdio.h>

#include "al/core.hpp"
#include "al/util/al_Font.hpp"
#include "al/util/ui/al_Parameter.hpp"
#include "al/util/al_AlloSphereSpeakerLayout.hpp"
#include "Gamma/SamplePlayer.h"
//#include "alloaudio/al_SoundfileBuffered.hpp"
//#include "al/core/sound/lbap.hpp"
#include "lbap.hpp"

using namespace al;

#define BLOCK_SIZE (1024)

//#define SKIP_FOR_DEBUG

int sampleRate = 44100;
int outputChannels = 0;
static std::atomic<float> *mPeaks;


ParameterServer paramServer("127.0.0.1",8080);
Parameter srcAzimuth("srcAzimuth","",0.1);
Parameter srcElevation("srcElevation","",-0.1);
Parameter sourceSound("sourceSound","",0);

float srcRadius = 6.0;

gam::SamplePlayer<> player;


std::string path = "/Users/sphere/code/nathan/Gamma/sounds/count.wav";
//SoundFileBuffered soundFileBuffered(path, true);

float *sfBuffer;

static Lbap *panner;


//struct SoundObject{

//    float *buffer;
//    float azimuth;
//    float elevation;

//    SoundObject(float azimuth = 0, float elevation = 0): azimuth(azimuth), elevation(elevation) {
//        buffer = (float*) calloc (BLOCK_SIZE,sizeof(float));
//    }
//};

//vector<SoundObject> soundObjects;

Vec3d convertCoords(Vec3d v){
    Vec3d returnVector;

    returnVector.x = -v.y;
    returnVector.y = v.z;
    returnVector.z = -v.x;
//    returnVector.x = v.x;
//    returnVector.y = v.z;
//    returnVector.z = -v.y;
    return returnVector;
}

class MyApp : public App
{
 public:
#ifndef SKIP_FOR_DEBUG
    Font mFont;
    Mesh mText;
#endif

    Mesh mSpeakerMesh;
    Light mLight;

//public:
    MyApp(){

#ifndef SKIP_FOR_DEBUG
        //mFont.load("allocore/share/fonts/VeraMoIt.ttf", 20);
//        mFont.load("data/VeraMoIt.ttf", 20);
//        addSurface(mText,0,4);
#endif
        mLight.pos(0,6,0);
        addSphere(mSpeakerMesh);
        nav().pos(0, 1.0, 20.0);
        start();
        player.load("/Users/primary1/Documents/code/AlloSystemTest/Gamma/sounds/water3.wav");
    }

    void onCreate(){
#ifndef SKIP_FOR_DEBUG
        mFont.load("data/VeraMoIt.ttf", 20);
#endif
    }


    void onDraw(Graphics& g){

        g.clear();
        g.blendAdd();

        //Draw the source
        g.pushMatrix();

//        double cosel = cos(srcElevation.get());
//        double x = sin(srcAzimuth.get()) * cosel * srcRadius;
//        double y = cos(srcAzimuth.get()) * cosel * srcRadius;
//        double z = sin(srcElevation.get()) * srcRadius;

        double cosel = cos(srcElevation.get());
        double x = cos(srcAzimuth.get()) * cosel * srcRadius;
        double y = sin(srcAzimuth.get()) * cosel * srcRadius;
        double z = sin(srcElevation.get()) * srcRadius;

        g.translate(convertCoords(Vec3d(x,y,z)));
        g.scale(0.1);
        g.color(1.0,0.0,0.0);
        g.draw(mSpeakerMesh);
        g.popMatrix();

        //Draw the speakers
        for(int i = 0; i < panner->speakerLayers.size(); ++i){

//            g.color(1.0,1.0,1.0);
            std::vector<SpeakerLBAP> aSpeakerLayer = panner->speakerLayers[i].speakers;
            for(int j = 0; j < aSpeakerLayer.size(); j++){
                g.pushMatrix();
//                if(i==0){
//                    g.color(1.0,0.0,0.0);
//                }else{
//                    g.color(1.0,1.0,1.0);
//                }
                g.color(1.0,1.0,1.0);
                g.translate(convertCoords(aSpeakerLayer[j].position));

                g.pushMatrix();
                float peak = mPeaks[aSpeakerLayer[j].channel].load();
                //peakIndex++;
                g.scale(0.02 + 0.04 * peak * 30);
                g.draw(mSpeakerMesh);
                g.popMatrix();

                //Draw Text Label
                //g.pushMatrix();
                g.scale(0.01);
                g.translate(0.0,10.0,0.0);


#ifndef SKIP_FOR_DEBUG
                mFont.render(g,std::to_string(aSpeakerLayer[j].channel));
               // mFont.write(mText,std::to_string(aSpeakerLayer[j].channel));
               // mFont.texture().bind();
               // g.draw(mText);
               // mFont.texture().unbind();
//                mFont.write(mText,std::to_string(aSpeakerLayer[j].channel));
//                mFont.texture().bind();
//                g.draw(mText);
//                mFont.texture().unbind();
#endif

                //g.popMatrix();
//                float peak = mPeaks[aSpeakerLayer[j].channel].load();
//                //peakIndex++;
//                g.scale(0.02 + 0.04 * peak * 30);
//                g.draw(mSpeakerMesh);
                g.popMatrix();

            }
        }
    }
};

static void audioCB(AudioIOData& io){

    static unsigned int t = 0;
    float srcSound = sourceSound.get();
    float srcBuffer[BLOCK_SIZE];

    //second src test
    float srcBuffer2[BLOCK_SIZE];

    //soundFileBuffered.read(sfBuffer,BLOCK_SIZE);

    while (io()) {

        player.loop();

        if(srcSound == 0){
            double sec = (t / io.fps());
            srcBuffer[io.frame()] = 0.5 * sin(sec*440*M_2PI);

//            srcBuffer2[io.frame()] = 0.5 * sin(sec*440*M_2PI);
            srcBuffer2[io.frame()] = player();

            t++;
        } else if(srcSound == 1){
            srcBuffer[io.frame()] = sfBuffer[io.frame()];
        }
    }

    Vec3d srcPos(srcAzimuth.get(),srcElevation.get(),srcRadius);

    panner->renderBuffer(io,srcPos,srcBuffer,BLOCK_SIZE);

    //second src test
    Vec3d srcPos2(-0.5,0.5,6.0);
    panner->renderBuffer(io,srcPos2,srcBuffer2,BLOCK_SIZE);

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

    sfBuffer = (float*) calloc (BLOCK_SIZE,sizeof(float));

    paramServer.registerParameter(srcAzimuth);
    paramServer.registerParameter(srcElevation);
    paramServer.registerParameter(sourceSound);

    AlloSphereSpeakerLayout alloLayout;
    panner = new Lbap(alloLayout,false);

    printf("layer1 %f \n",panner->speakerLayers[1].elevation);

    int numSpeakers = 0;
    int highestChannel = 0;
    for(int i = 0 ; i < panner->speakerLayers.size(); i++){

        std::vector<SpeakerLBAP> aSpeakerLayer = panner->speakerLayers[i].speakers;
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


    AudioIO audioIO; //(BLOCK_SIZE, sampleRate, audioCB, 0, outputChannels, 1);
    audioIO.init(audioCB,nullptr,BLOCK_SIZE,sampleRate,outputChannels,1);
    //audioIO.device(AudioDevice("ECHO X5"));
    audioIO.start();

    audioIO.print();



    MyApp().start();

    getchar();
    return 0;
}
