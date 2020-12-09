#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float fGapSize;
    int nGripperPos;
    double fPosePerMM;
}stGripperPos;
static vector<stGripperPos> arGripperPos;

void InitGripperPosVal()
{
    stGripperPos tmpGP;
    tmpGP.fGapSize = 0.081;
    tmpGP.nGripperPos = 38000;
    tmpGP.fPosePerMM = 0;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.074;
    tmpGP.nGripperPos = 35000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.061;
    tmpGP.nGripperPos = 30000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.048;
    tmpGP.nGripperPos = 25000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.034;
    tmpGP.nGripperPos = 20000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.020;
    tmpGP.nGripperPos = 15000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.006;
    tmpGP.nGripperPos = 10000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.000;
    tmpGP.nGripperPos = 5000;
    arGripperPos.push_back(tmpGP);

    tmpGP.fGapSize = 0.000;
    tmpGP.nGripperPos = 0;
    arGripperPos.push_back(tmpGP);

    int nNumGP = arGripperPos.size();
    for(int i=1;i<nNumGP;i++)
    {
        double fDiffSize = fabs(arGripperPos[i].fGapSize - arGripperPos[i-1].fGapSize);
        int nDiffPos = fabs(arGripperPos[i].nGripperPos - arGripperPos[i-1].nGripperPos);
        arGripperPos[i].fPosePerMM = fDiffSize/nDiffPos;
        //ROS_WARN("i=%d fPosePerMM =%f",i,arGripperPos[i].fPosePerMM);
    }
    arGripperPos[0].fPosePerMM = arGripperPos[1].fPosePerMM;
}

// 手爪位置计算
int CalGripperPos(float inGapSize)
{
    int nNumGP = arGripperPos.size();
    int nRetGripperPos = 0;
    if(nNumGP > 0)
    {
        int nIndexGP = nNumGP-1;
        for(int i=0;i<nNumGP;i++)
        {
            if(inGapSize >= arGripperPos[i].fGapSize)
            {
                nIndexGP = i;
                break;
            }
        }
        double fDiffGapSize = fabs(inGapSize - arGripperPos[nIndexGP].fGapSize);
        int nDiffGripperPos = (fDiffGapSize/arGripperPos[nIndexGP].fPosePerMM);
        nRetGripperPos = arGripperPos[nIndexGP].nGripperPos + nDiffGripperPos;
    }
    return nRetGripperPos;
}