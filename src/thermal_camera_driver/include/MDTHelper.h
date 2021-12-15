#ifndef MDTHELPER_H
#define MDTHELPER_H

struct MDT {
    unsigned char * jpgBuf;
    int jpgSize;
    unsigned char * ddtBuf;
    int ddtSize;
    unsigned char * visBuf;
    int visSize;
    unsigned char * labBuf;
    int labSize;
    unsigned char * txtBuf; // UTF8格式
    int txtSize;
    unsigned char * audBuf;
    int audSize;
};

enum RoiType
{
    RoiNone = -1,
    RoiPoint = 0,
    RoiLine = 1,
    RoiRect = 2,
    RoiCircle = 3,
    RoiEllipse = 4
};
//2.如下结构体数组
struct ROI {
    RoiType type;		//0-点；1-线；2-矩形；3-圆；4-椭圆；
    int targetType;		//代表温度，0-max；1-min；2-ave 3-diff
    int maxTemp;		//最高温，未经修正，mc
    int minTemp;		//最低温，未经修正，mc
    int aveTemp;		//平均温，未经修正，mc
    int maxPos;			//最高温位置，fpa坐标系
    int minPos;			//最低温位置，fpa坐标系
    int x0;				//fpa坐标系，x0 < x1
    int y0;				//fpa坐标系，y0 > y1
    int x1;				//fpa坐标系，x0 < x1
    int y1;				//fpa坐标系，y0 > y1
    int color;			//颜色, 0x00RRGGBB
    float emissivity;	//辐射率
    int lowerAlarmTemp;	//报警温度下限
    int upperAlarmTemp;	//报警温度上限
    char name[64];		//名称,UTF8格式
    int reserved[33];
};

class MDTHelper
{
public:
    MDTHelper();
    ~MDTHelper();

    bool load(const char * pathName);
    void unload();
    bool save(const char * pathName);

    void set(const struct MDT * mdt);

    void get(struct MDT * mdt);

    bool setJpg(unsigned char * jpgBuf, int size, bool isCopy);

    bool setDDT(unsigned char * ddtBuf, int size, bool isCopy);

    bool setVisible(unsigned char * visBuf, int size, bool isCopy);

    bool setLabel(struct ROI * labBuf, int size, bool isCopy);

    bool setText(unsigned char * txtBuf, int size, bool isCopy); // UTF8格式

    bool setAudio(unsigned char * audBuf, int size, bool isCopy);

    int getJpg(unsigned char ** jpgBuf);

    int getDDT(unsigned char  ** ddtBuf);

    int getVisible(unsigned char ** visBuf);

    int getLabel(struct ROI ** labBuf);

    int getText(unsigned char ** txtBuf);

    int getAudio(unsigned char ** audBuf);



private:
    void free();
    bool isSizeValid(int val);

private:
    unsigned char * mJpgBuf;
    int mJpgSize;
    bool mIsJpgAlloc;

    unsigned char * mDDTBuf;
    int mDDTSize;
    bool mIsDDTAlloc;

    unsigned char * mVisBuf;
    int mVisSize;
    bool mIsVisAlloc;

    unsigned char * mLabBuf;
    int mLabSize;
    bool mIsLabAlloc;

    unsigned char * mTxtBuf;
    int mTxtSize;
    bool mIsTxtAlloc;

    unsigned char * mAudBuf;
    int mAudSize;
    bool mIsAudAlloc;
};

#endif // MDTHELPER_H
