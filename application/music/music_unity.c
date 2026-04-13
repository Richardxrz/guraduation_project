#include "music_unity.h"

#include "bsp_buzzer.h"
#include "music.h"
#include "stm32f4xx_hal.h"

// clang-format off
//定义低音  
#define A1  131
#define A2  147
#define A3  165
#define A4  175
#define A5  196
#define A6  220
#define A7  247
  
//定义中音  
#define B1  262
#define B2  296
#define B3  330
#define B4  349
#define B5  392
#define B6  440
#define B7  494
  
//定义高音  
#define C1  523
#define C2  587
#define C3  659
#define C4  698
#define C4p 741
#define C5  784
#define C6  880
#define C7  988
  
//定义高二度  
#define D1  1047
#define D2  1175
#define D3  1319
#define D4  1397
#define D5  1568
#define D6  1760
#define D7  1976
  
//定义节拍  
#define OneBeat   200//一拍子两个1beat 
#define HalfBeat  100
// clang-format on

#define NOTE_NUM 200
static Note Notes[NOTE_NUM];  // Array of notes
static MusicInfo_s MUSIC_INFO;

/*-------------------- User functions --------------------*/

MusicInfo_s MusicUnityInit(void)
{
    MUSIC_INFO.notes = Notes;

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 2 + HalfBeat);
    WRITE_NOTE(D1, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(0, OneBeat + HalfBeat);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(B6, HalfBeat / 2);
    WRITE_NOTE(C1, HalfBeat * 3);
    WRITE_NOTE(C2, HalfBeat / 2);
    WRITE_NOTE(C3, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 2 + HalfBeat);
    WRITE_NOTE(C5, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(D1, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(0, OneBeat + HalfBeat);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 2 + HalfBeat);
    WRITE_NOTE(D1, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(0, OneBeat + HalfBeat);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(D3, HalfBeat / 2);
    WRITE_NOTE(D2, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat / 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 4);

    WRITE_NOTE(B6, HalfBeat / 2);
    WRITE_NOTE(C1, HalfBeat * 3);
    WRITE_NOTE(C2, HalfBeat / 2);
    WRITE_NOTE(C3, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat / 2);
    WRITE_NOTE(C6, OneBeat * 2 + HalfBeat);
    WRITE_NOTE(C5, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(D1, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(0,OneBeat + 150);
    WRITE_NOTE(C3, OneBeat * 2);
    WRITE_NOTE(0,OneBeat + 150);

    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C5, OneBeat * 2);
    WRITE_NOTE(C3, OneBeat * 2);

    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(D6, HalfBeat);
    WRITE_NOTE(D5, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);

    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D5, HalfBeat);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(D1, OneBeat * 2);

    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D5, HalfBeat);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);
    WRITE_NOTE(D1, OneBeat * 2);

    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(D1, HalfBeat);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C5, OneBeat * 2);
    WRITE_NOTE(C3, OneBeat * 2);

    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, OneBeat * 2);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(C5, HalfBeat);
    WRITE_NOTE(C6, HalfBeat * 3);
    WRITE_NOTE(D6, HalfBeat);
    WRITE_NOTE(D5, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D2, OneBeat * 2);

    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D1, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D1, HalfBeat * 3);
    WRITE_NOTE(D2, HalfBeat);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);
    WRITE_NOTE(D3, OneBeat * 2);

    return MUSIC_INFO;
}
