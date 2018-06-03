#include "al/core/sound/al_AudioScene.hpp"

namespace al {

struct SpeakerLBAP
{
    int channel;


    float azimuth;
    float elevation;
    Vec3f position;

    SpeakerLBAP *leftSpeaker;
    SpeakerLBAP *rightSpeaker;


    //TODO: isLeftHanded - change name (intended to take into accound
    SpeakerLBAP(int channel=0,float angle=0.0,float elevation=0.0, float radius=1.0, bool isLeftHanded=true): elevation(elevation), channel(channel) {

        azimuth = toRad(angle);

        if(azimuth < 0.0){
            azimuth += M_2PI;
        }

//        if(!isLeftHanded){
//            azimuth = M_2PI - azimuth;
//        }

//        double cosel = cos(toRad(elevation));
//        double x = sin(azimuth) * cosel * radius;
//        double y = cos(azimuth) * cosel * radius;
//        double z = sin(toRad(elevation)) * radius;

        double cosel = cos(toRad(elevation));
        double x = cos(azimuth) * cosel * radius;
        double y = sin(azimuth) * cosel * radius;
        double z = sin(toRad(elevation)) * radius;

        position = Vec3d(x,y,z);
        this->elevation = toRad(elevation);
    }

    static double toRad(double d){ return d*M_PI/180.; }
};

struct SpeakerLayer {

    //TODO: Maybe make private
    float elevation;

    std::vector<SpeakerLBAP> speakers;


    //TODO: Add checks for 0-2PI or 0-360
    void setElevationUsingDegrees(float elev){elevation = toRad(elev);}
    void setElevationUsingRadians(float elev){elevation = elev;}

    static bool speakerSort(SpeakerLBAP const &first, SpeakerLBAP const &second){
        return first.azimuth < second.azimuth;
    }

    int initLayer(){
        std::sort(speakers.begin(),speakers.end(),&SpeakerLayer::speakerSort);
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

class Lbap : public Spatializer{
public:
    Lbap(const SpeakerLayout &sl, bool isLeftHanded = false);

    //virtual void compile(Listener& listener) override;

    virtual void renderSample(AudioIOData& io, const Pose& reldir, const float& sample, const int& frameIndex) override;
    virtual void renderBuffer(AudioIOData& io, const Pose& reldir, const float *samples, const int& numFrames) override;

    SpeakerLayer slBottom, slMiddle, slTop;
    std::vector<SpeakerLayer> speakerLayers;

private:
    bool isLeftHanded;
    Listener* mListener;


    float diffOfAngles(float angle1, float angle2);

    void findSpeakerPair(float sourceAzimuth, SpeakerLayer *layer, SpeakerLBAP*& leftSpeaker, SpeakerLBAP*& rightSpeaker);

};


}
