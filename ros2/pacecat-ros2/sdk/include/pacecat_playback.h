#include"alarm.h"
#include"algorithmAPI.h"
#include"parser.h"
#include"reader.h"
#include"usererror.h"

class PaceCatPlaybackDriver
{
public:
    PaceCatPlaybackDriver();
    ~PaceCatPlaybackDriver();

    //playback
    ArgDataPlayback& getArgDataPlayback() { return m_argdataplayback; }
    void openPlaybackThread();
    PubHub* getPlaybackHub() { return m_hubplayback; }
    bool checkPlaybackIsRun();
    int GetPlaybackAllFans(PubHub* pub, ArgDataPlayback &argdata);
    double ROSAng(double ang);
    int GetCount(std::vector<DataPoint> data, double min_deg, double max_deg, double &min_pos, double &max_pos);

private:
    //playback
    HReader m_readerplayback;
    Parser *m_parserplayback;
    PubHub *m_hubplayback;
    ArgDataPlayback m_argdataplayback;
};