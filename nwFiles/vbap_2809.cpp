#define GAMMA_H_INC_ALL         // define this to include all header files
#define GAMMA_H_NO_IO           // define this to avoid bringing AudioIO from Gamma

#include "Gamma/Gamma.h"
#include "al/core.hpp"
#include "al/core/sound/al_Speaker.hpp"
#include "al/util/ui/al_Parameter.hpp"
#include "al/util/ui/al_ParameterServer.hpp"
#include "Gamma/SamplePlayer.h"
#include "al/util/ui/al_ControlGUI.hpp"

#include <atomic>
#include <vector>

#define SAMPLE_RATE 44100
#define BLOCK_SIZE (2048)

using namespace al;
using namespace std;

ParameterServer paramServer("127.0.0.1",8080);
Parameter srcAzimuth("srcAzimuth","",-0.5,"",-1.0*M_PI,M_PI);
ParameterBool updatePanner("updatePanner","",0.0);


ParameterBool useRamp("useRamp","",0);
Parameter rampStartAzimuth("rampStartAzimuth","",-0.5,"",-1.0*M_PI,M_PI);
Parameter rampEndAzimuth("rampEndAzimuth","",0.5,"",-1.0*M_PI,M_PI);
Parameter rampDuration("rampDuration", "",1.0,"",0.0,5.0);

float radius = 5.0;

Parameter sourceSound("sourceSound","",0.0,"",0.0,3.0);
Parameter soundFileIdx("soundFileIdx","",0.0);

ParameterBool sampleWise("sampleWise","",0.0);

ParameterBool swise("swise");
ParameterBool a("a","",0,"",0,1);

Parameter masterGain("masterGain","",0.5,"",0.0,1.0);
ParameterBool soundOn("soundOn","",0.0);

ParameterBool triggerRamp("triggerRamp","",0.0);

//int sourceSound = 1;
//int soundFileIdx = 1;

class SpeakerV: public Speaker {
public:

    ParameterBool *enabled;
    std::string oscTag;
    float aziInRad;

    SpeakerV(int chan, float az=0.f, float el=0.f, int gr=0, float rad=1.f, float ga=1.f)

    {
        deviceChannel = chan;
        azimuth= az;
        elevation = el;
        group = gr;
        radius = rad;
        gain = ga;
        aziInRad = toRad(az);

        oscTag = "speaker"+ std::to_string(deviceChannel) + "/enabled";
        enabled = new ParameterBool(oscTag,"",1.0);
        enabled->setHint("latch",1.f);
        paramServer.registerParameter(*enabled);
    }
};

std::vector<SpeakerV> speakers;
std::vector<SpeakerV> enabledSpeakers;
Mat<2,double> matrix;


struct Ramp {

    unsigned int startSample, endSample;
    float startAzimuth = -2.0;
    float endAzimuth = 2.0;
    float duration;
    bool done = false;
    bool running = false;

    //Parameter *trigger;//("triggerRamp","",0.0);
    bool trigger = false;

    Ramp(){
        //trigger = new Parameter("triggerRamp","",0.0);
        //paramServer.registerParameter(*trigger);
    }

    void set(float startAzi, float endAzi, float dur){
        startAzimuth = startAzi;
        endAzimuth = endAzi;
        duration = dur;
    }

    void start(unsigned int startSamp){
        //unsigned int i=static_cast<unsigned int>(duration);
        startSample = startSamp;
        endSample = startSamp +  duration *SAMPLE_RATE;
        running = true;
        done = false;
        cout << startSamp << " endSamp: " << endSample << endl;
    }

    float next(unsigned int sampleNum){

        if(trigger){
            cout << "linRampTriggered" << endl;
            trigger = false;
            start(sampleNum);
        }

        if(!done && !running){
            return startAzimuth;
        }else if(!done && running){

            if(sampleNum > endSample){
                sampleNum = endSample;
                done = true;
                running = false;
            }
            float val = (((sampleNum - startSample) * (endAzimuth - startAzimuth)) / (endSample - startSample)) + startAzimuth;
            return val;
        } else {
            return endAzimuth;
        }
    }
};

bool speakerSort(SpeakerV const &first, SpeakerV const &second){
    return first.azimuth < second.azimuth;
}

void initPanner(){
    enabledSpeakers.clear();
    for(int i = 0; i < speakers.size(); i ++){
        if(speakers[i].enabled->get() > 0.5){
            enabledSpeakers.push_back(speakers[i]);
        }
    }
    std::sort(enabledSpeakers.begin(),enabledSpeakers.end(),&speakerSort);
}

class MyApp : public App
{
public:
    Mesh mSpeakerMesh;
    vector<Mesh> mVec;
    vector<int>  sChannels;

    SpeakerLayout speakerLayout;
    atomic<float> *mPeaks {nullptr};
    float speedMult = 0.03f;

    Vec3d srcpos {0.0,0.0,0.0};

    Ramp linearRamp;
    vector<gam::SamplePlayer<>*> samplePlayers;
    SearchPaths searchpaths;


    ControlGUI parameterGUI;

    MyApp()
    {

        //ImGui::Checkbox("CheckBox")
        //a.setHint("checkbox",1.0f);

        parameterGUI << soundOn << srcAzimuth << updatePanner << triggerRamp << rampStartAzimuth << rampEndAzimuth << rampDuration << sourceSound << soundFileIdx << sampleWise << masterGain << useRamp <<  a;
//        parameterServer() << srcAzimuth << updatePanner << rampStartAzimuth << rampEndAzimuth << rampDuration << sourceSound << soundFileIdx << sampleWise << swise;

        paramServer << srcAzimuth << updatePanner << triggerRamp << rampStartAzimuth << rampEndAzimuth << rampDuration << sourceSound << soundFileIdx << sampleWise << masterGain << useRamp << soundOn;





        soundOn.setHint("latch", 1.f);
        sampleWise.setHint("latch", 1.f);
        useRamp.setHint("latch",1.f);

        sourceSound.setHint("intcombo",1.f);

        //rampStartAzimuth.setHint("intcombo",1.f);

        triggerRamp.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                cout << "Ramp triggered" << endl;
                linearRamp.trigger = true;
            }

        });

        updatePanner.registerChangeCallback([&](float val){
            if(val == 1.f){ // is this correct way to check?
                cout << "panner Updated" << endl;
            }
            initPanner();
        });

        swise.registerChangeCallback([&](float newAzi){
            cout << " swise: " << swise.get() << endl;
        });


        swise.setHint("latch",1.f);


        //parameterGUI.registerParameterMeta()
        rampStartAzimuth.registerChangeCallback([&](float newAzi){
            linearRamp.startAzimuth = newAzi;
        });

        rampEndAzimuth.registerChangeCallback([&](float newAzi){
            linearRamp.endAzimuth = newAzi;
        });

        rampDuration.registerChangeCallback([&](float newDur){
            linearRamp.duration = newDur;
        });
    }


    void createMatrix(SpeakerV &left, SpeakerV &right){
        matrix.set(left.vec().x,left.vec().y,
                   right.vec().x,right.vec().y
                   );
    }

    Vec3d ambiSphericalToOGLCart(float azimuth, float radius){
        Vec3d ambiSpherical;
        float elevation = 0.0;

        //find xyz in cart audio coords
        float x = radius * cos(elevation) * cos(azimuth);
        float y = radius * cos(elevation) * sin(azimuth);
        float z = radius * sin(elevation);

        //convert to open_gl cart coords
        ambiSpherical[0] = -y;
        ambiSpherical[1] = z;
        ambiSpherical[2] = -x;

        return ambiSpherical;
    }

    void openGLCartToAmbiCart(Vec3f &vec){
        Vec3f tempVec = vec;
        vec[0] = tempVec.z*-1.0;
        vec[1] = tempVec.x*-1.0;
        vec[2] = tempVec.y;
    }

    Vec3d calcGains(AudioIOData &io, int &speakerChan1, int &speakerChan2){
        float srcAzi = srcAzimuth.get();

        srcpos = ambiSphericalToOGLCart(srcAzi,radius);

        Vec3f ambiCartSrcPos = srcpos;
        openGLCartToAmbiCart(ambiCartSrcPos);

        std::sort(enabledSpeakers.begin(),enabledSpeakers.end(),&speakerSort);

        Vec3d gains(0.,0.,0.);
//        int speakerChan1, speakerChan2;

        float speakSrcAngle,linearDistance;

        //check if source is beyond the first or last speaker
        if(srcAzi < enabledSpeakers[0].aziInRad){
            speakerChan1 = enabledSpeakers[0].deviceChannel;
            speakerChan2 = enabledSpeakers[0+1].deviceChannel;
            speakSrcAngle = fabsf(enabledSpeakers[0].aziInRad - srcAzi);
//            linearDistance = 2.0*radius*cos((M_PI - speakSrcAngle)/2.0);
//            gains.x = 1.f / (1.f + linearDistance);

            gains.x = 1.f / radius * (M_PI - speakSrcAngle);

        } else if(srcAzi > enabledSpeakers[enabledSpeakers.size()-1].aziInRad){
            speakerChan1 = enabledSpeakers[enabledSpeakers.size()-2].deviceChannel;//set to speaker before last
            speakerChan2 = enabledSpeakers[enabledSpeakers.size()-1].deviceChannel;
            speakSrcAngle = fabsf(enabledSpeakers[enabledSpeakers.size()-1].aziInRad - srcAzi);
            linearDistance = 2.0*radius*cos((M_PI - speakSrcAngle)/2.0);
            //gains.y = 1.f / (1.f + linearDistance);
            gains.y = 1.f / radius * (M_PI - speakSrcAngle);

        } else{//Source between first and last speakers

            for(int i = 0; i < enabledSpeakers.size()-1; i++){
                speakerChan1 = enabledSpeakers[i].deviceChannel;
                speakerChan2 = enabledSpeakers[i+1].deviceChannel;

                if(srcAzi == enabledSpeakers[i].aziInRad ){
                    gains.x = 1.0;
                    break;
                }else if(srcAzi > enabledSpeakers[i].aziInRad && srcAzi < enabledSpeakers[i+1].aziInRad){

                    createMatrix(enabledSpeakers[i],enabledSpeakers[i+1]);
                    invert(matrix);
                    for (unsigned i = 0; i < 2; i++){
                        for (unsigned j = 0; j < 2; j++){
                            gains[i] += ambiCartSrcPos[j] * matrix(j,i);
                        }
                    }

                    gains = gains.normalize();
                    break;

                } else if(srcAzi == enabledSpeakers[i+1].aziInRad){
                    gains.y = 1.0;
                    break;
                }
            }
        }

        return gains;
    }

    void renderBuffer(AudioIOData &io, const float *buffer){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io, speakerChan1, speakerChan2);
        for(int i = 0; i < io.framesPerBuffer(); i++){
            io.out(speakerChan1,i) += buffer[i]*gains[0];
            io.out(speakerChan2,i) += buffer[i]*gains[1];
        }
    }

    void renderSample(AudioIOData &io, const float &sample, const int &frameIndex){
        int speakerChan1, speakerChan2;
        Vec3d gains = calcGains(io, speakerChan1, speakerChan2);
        io.out(speakerChan1,frameIndex) += sample * gains[0];
        io.out(speakerChan2,frameIndex) += sample * gains[1];
    }

    void onInit() override {

        searchpaths.addAppPaths();
        searchpaths.addRelativePath("../sounds");

        samplePlayers.push_back(new gam::SamplePlayer<>(searchpaths.find("lowBoys.wav").filepath().c_str()));
        samplePlayers.push_back(new gam::SamplePlayer<>(searchpaths.find("devSamples.wav").filepath().c_str()));

        linearRamp.set(-2.0,2.0,rampDuration.get());


        for (int i = 0; i < 20; i++){
            speakers.push_back(SpeakerV(i,-90.0 + (10.0*i),0.0,0,5.0));
        }

//        speakers.push_back(SpeakerV(0,-45.0,0.0,0,5.0));
//        speakers.push_back(SpeakerV(1,-20.0,0.0,0,5.0));
//        speakers.push_back(SpeakerV(2,0.0,0.0,0,5.0));
//        speakers.push_back(SpeakerV(3,20.0,0.0,0,5.0));
//        speakers.push_back(SpeakerV(4,45.0,0.0,0,5.0));

        speakers[1].enabled->set(0.0);

        initPanner();

        for(int i = 0; i < speakers.size(); i++){
            parameterGUI << speakers[i].enabled;
            //speakers[i].enabled->setHint("latch",1.f);
        }

        int highestChannel = 0;
        for(SpeakerV s:speakers){
            if((int) s.deviceChannel > highestChannel){
                highestChannel = s.deviceChannel;
            }
        }

        audioIO().close();
        audioIO().channelsOut(highestChannel + 1);
        audioIO().open();

        mPeaks = new atomic<float>[highestChannel + 1];

        addSphere(mSpeakerMesh, 1.0, 5, 5);

    }

    void onCreate() override {
        nav().pos(0, 1, 20);
        parameterGUI.init();
    }

    void onAnimate( double dt) override {
//        static int cnt = 0;
//        if(cnt % 10 == 0){
//            cout << swise.get() << endl;
//        }
//        cnt++;


        //nav().faceToward(Vec3f(0,0,0)); // Always face origin
    }

    virtual void onSound(AudioIOData &io) override {

//        if(updatePanner.get() > 0.5){
//            cout << "updatePanner triggered" << endl;
//            updatePanner.set(0.0);
//            initPanner();
//        }

        static unsigned int t = 0;
        double sec;
        float srcBuffer[BLOCK_SIZE];

        float mGain = masterGain.get();
        gam::Sync::master().spu(audioIO().fps());

        if(soundOn.get() > 0.5){
            while (io()) {
                int i = io.frame();
                if(sourceSound.get() == 0){
                    float env = (22050 - (t % 22050))/22050.0;
                    srcBuffer[i] = mGain * rnd::uniform() * env;
                } else if(sourceSound.get() ==1){
                    gam::SamplePlayer<> *player = samplePlayers[soundFileIdx];
                    if(player->done()){
                        player->reset();
                    }
                    srcBuffer[i] = mGain * player->operator ()();
                }
                ++t;

                if(useRamp.get()){
                    srcAzimuth.set(linearRamp.next(t));
                }

                if(sampleWise.get() == 1.f){
                    renderSample(io,srcBuffer[i],i);
                }
            }

            if(sampleWise.get() == 0.f){
                renderBuffer(io,srcBuffer);
            }
        }

        for (int i = 0; i < speakers.size(); i++) {
            mPeaks[i].store(0);
        }
        for (int speaker = 0; speaker < speakers.size(); speaker++) {
            float rms = 0;
            int deviceChannel = speakers[speaker].deviceChannel;
            for (int i = 0; i < io.framesPerBuffer(); i++) {
                if(deviceChannel < io.channelsOut()) {
                    float sample = io.out(deviceChannel, i);
                    rms += sample * sample;
                }
            }
            rms = sqrt(rms/io.framesPerBuffer());
            mPeaks[deviceChannel].store(rms);
        }
    }

    virtual void onDraw(Graphics &g) override {

        g.clear(0);
        g.blending(true);
        g.blendModeAdd();

        //Draw the source
        g.pushMatrix();
        g.translate(srcpos);
        g.scale(0.3);
        g.color(0.4,0.4, 0.4, 0.5);
        g.draw(mSpeakerMesh);
        g.popMatrix();

        // Draw line
        Mesh lineMesh;
        lineMesh.vertex(0.0,0.0, 0.0);
        lineMesh.vertex(srcpos.x,0.0, srcpos.z);
        lineMesh.vertex(srcpos);
        lineMesh.index(0);
        lineMesh.index(1);
        lineMesh.index(1);
        lineMesh.index(2);
        lineMesh.index(2);
        lineMesh.index(0);
        lineMesh.primitive(Mesh::LINES);
        g.color(1);
        g.draw(lineMesh);

        //Draw the speakers
        for(int i = 0; i < speakers.size(); ++i){
            int devChan = speakers[i].deviceChannel;
            g.pushMatrix();
            g.translate(speakers[i].vecGraphics());
            float peak = mPeaks[devChan].load();
            g.scale(0.02 + peak * 6);
            g.polygonLine();
            g.color(1);
            g.draw(mSpeakerMesh);
            g.popMatrix();
        }

        parameterGUI.draw(g);
    }
};

int main(){
    MyApp app;
    AudioDevice::printAll();

     //audioRate audioBlockSize audioOutputs audioInputs device

    app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 60, 0, -1);

    // Use this for 2809                   **CHANGE AUDIO INPUT TO 0 **
    //app.initAudio(SAMPLE_RATE, BLOCK_SIZE, 2, -1, AudioDevice("Aggregate Device").id());

    // Use this for sphere
    //    app.initAudio(44100, BLOCK_SIZE, -1, -1, AudioDevice("ECHO X5").id());

    app.start();
    return 0;
}
