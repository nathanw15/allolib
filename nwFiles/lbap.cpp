#include "lbap.hpp"

namespace al {


Lbap::Lbap(const SpeakerLayout &sl, bool isLeftHanded):	Spatializer(sl), isLeftHanded(isLeftHanded){


    //1.) Find the number of layers and elevations of each layer
    //2.) Sort elevations lowest to highest
    //3.) Create speaker layer for each elevation
    //4.) Add each speaker layer to speakerLayers
    //5.) Assign each speaker to a speaker layer (warn or quit if a speaker cant be assigned)
    //6.) Sort speakers in each speaker layer
    //7.) Find left and right speakers of each speaker

    //TODO: Make this work for generic speaker layout

    //    SpeakerLayer slBottom(-32.5);
    //    SpeakerLayer slMiddle(0.0);
    //    SpeakerLayer slTop(41.0);
    slBottom.setElevationUsingDegrees(-32.5);
    slMiddle.setElevationUsingDegrees(0.0);
    slTop.setElevationUsingDegrees(41.0);

    float radius = 5.0;

    Speakers alloSpeakers = sl.speakers();

    //in radians
    float elevationTolerance = 0.01745;

    for(int i = 0; i < alloSpeakers.size();i++){
        Speaker sp = alloSpeakers[i];

        float elevationRad = sp.elevation * M_PI/180.0;

        if(elevationRad > slBottom.elevation-elevationTolerance && elevationRad < slBottom.elevation + elevationTolerance){
            slBottom.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,isLeftHanded));
        }else if(elevationRad > slMiddle.elevation-elevationTolerance && elevationRad < slMiddle.elevation + elevationTolerance){
            slMiddle.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,isLeftHanded));
        }else if(elevationRad > slTop.elevation-elevationTolerance && elevationRad < slTop.elevation + elevationTolerance){
            slTop.speakers.push_back(SpeakerLBAP(sp.deviceChannel,sp.azimuth,sp.elevation,radius,isLeftHanded));
        }
    }

    slBottom.initLayer();
    slMiddle.initLayer();
    slTop.initLayer();

    speakerLayers.push_back(slBottom);
    speakerLayers.push_back(slMiddle);
    speakerLayers.push_back(slTop);

}


float Lbap::diffOfAngles(float angle1, float angle2){
    return M_PI-fabsf(fabsf(angle1-angle2)-M_PI);
}

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

void Lbap::findSpeakerPair(float sourceAzimuth, SpeakerLayer *layer, SpeakerLBAP*& leftSpeaker, SpeakerLBAP*& rightSpeaker){

    for (int i = 0; i < layer->speakers.size(); i++){
        if(isAngleBetween(sourceAzimuth,layer->speakers[i].azimuth,layer->speakers[i].rightSpeaker->azimuth)){
            leftSpeaker = &layer->speakers[i];
            rightSpeaker = layer->speakers[i].rightSpeaker;
            break;
        }
    }
}

void spherToCartCoords(Vec3d &vec){

    float z = vec[2]*sin(vec[1]);
    float a = vec[2]*cos(vec[1]);
    float x = a*cos(vec[0]);
    float y = a*sin(vec[0]);

    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
}

void cartToSphericalCoords(Vec3d &vec){

    float radius = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
    float elevation  = asin(vec.z/radius);
    float azimuth = atan2(vec.y,vec.x);

    vec[0] = azimuth;
    vec[1] = elevation;
    vec[2] = radius;

}

void Lbap::compile(Listener& listener){
    this->mListener = &listener;
}


void Lbap::renderBuffer(AudioIOData &io, const Pose &reldir, const float *samples, const int &numFrames){
    //std::vector<float> out(outputChannels,0.0);

    //TREATING reldir as spherical Coords here! CHANGE


    Vec3d cords = reldir.vec();



    //Rotate vector according to listener-rotation
    //    Quatd srcRot = reldir.quat();
    //    cords = srcRot.rotate(cords);
    //    cords = Vec4d(cords.x, cords.z, cords.y);

    // Vec3d cords = reldir;

    //Find another way to do this
    spherToCartCoords(cords);
    cartToSphericalCoords(cords);

    float srcAzi = cords[0];
    float srcElev = cords[1];

    while(srcAzi < 0){
        srcAzi += M_2PI;
    }

    if(srcElev < speakerLayers[0].elevation || srcElev > speakerLayers[speakerLayers.size()-1].elevation){
        //src elevation is above the top or below the bottom layer
        SpeakerLBAP *left, *right;

        if(srcElev < speakerLayers[0].elevation){
            findSpeakerPair(srcAzi,&speakerLayers[0],left,right);
        } else {
            findSpeakerPair(srcAzi,&speakerLayers[speakerLayers.size()-1],left,right);
        }

        float dist = diffOfAngles(srcAzi,left->azimuth) / diffOfAngles(right->azimuth,left->azimuth);

        for(int i = 0; i < numFrames; i++){
            io.out(left->channel,i) = cos(dist*M_PI_2)*samples[i];
            io.out(right->channel,i)= sin(dist*M_PI_2)*samples[i];
        }
        return;
    }

//    if(srcElev < speakerLayers[0].elevation){
//        //Below lowest layer

//        SpeakerLBAP *left, *right;
//        findSpeakerPair(srcAzi,&speakerLayers[0],left,right);
//        float dist = diffOfAngles(srcAzi,left->azimuth) / diffOfAngles(right->azimuth,left->azimuth);

//        for(int i = 0; i < numFrames; i++){
//            io.out(left->channel,i) = cos(dist*M_PI_2)*samples[i];
//            io.out(right->channel,i)= sin(dist*M_PI_2)*samples[i];
//        }
//        return;

//    }else if(srcElev > speakerLayers[speakerLayers.size()-1].elevation){
//        //Above highest layer

//        SpeakerLBAP *left, *right;
//        findSpeakerPair(srcAzi,&speakerLayers[speakerLayers.size()-1],left,right);
//        float dist = diffOfAngles(srcAzi,left->azimuth) / diffOfAngles(right->azimuth,left->azimuth);

//        for(int i = 0; i < numFrames; i++){
//            io.out(left->channel,i) = cos(dist*M_PI_2)*samples[i];
//            io.out(right->channel,i)= sin(dist*M_PI_2)*samples[i];
//        }
//        return;
//    }

    SpeakerLayer *bottom;
    SpeakerLayer *upper;

    SpeakerLBAP *bottomLeft, *bottomRight, *upperLeft, *upperRight;

    //Find which layers contain the source
    for(int i = 0; i < speakerLayers.size()-1; i++){
        if(isAngleBetween(srcElev,speakerLayers[i].elevation,speakerLayers[i+1].elevation)){
            bottom = &speakerLayers[i];
            upper = &speakerLayers[i+1];
            break;
        }
    }

    //Find which speakers contain the source in the bottom and upper layers
    findSpeakerPair(srcAzi,bottom,bottomLeft,bottomRight);
    findSpeakerPair(srcAzi,upper,upperLeft,upperRight);

    float belowDist = (srcElev - bottom->elevation) / (upper->elevation - bottom->elevation);

    float aboveAmp = cos(belowDist*M_PI_2);
    float belowAmp = sin(belowDist*M_PI_2);

    float blDist = diffOfAngles(srcAzi,bottomLeft->azimuth) / diffOfAngles(bottomRight->azimuth,bottomLeft->azimuth);
    float alDist = diffOfAngles(srcAzi,upperLeft->azimuth) / diffOfAngles(upperRight->azimuth,upperLeft->azimuth);

    for(int i = 0; i < numFrames; i++){

        //QUESTION: would calculating the gains outside of this loop be more efficient or is it not necessary?
        io.out(bottomLeft->channel,i) = cos(blDist*M_PI_2)*cos(belowAmp*M_PI_2)*samples[i];
        io.out(bottomRight->channel,i) = sin(blDist*M_PI_2)*cos(belowAmp*M_PI_2)*samples[i];

        io.out(upperLeft->channel,i) = cos(alDist*M_PI_2)*cos(aboveAmp*M_PI_2)*samples[i];
        io.out(upperRight->channel,i) = sin(alDist*M_PI_2)*cos(aboveAmp*M_PI_2)*samples[i];

    }

}

void Lbap::renderSample(AudioIOData &io, const Pose &reldir, const float &sample, const int &frameIndex){


}



}
